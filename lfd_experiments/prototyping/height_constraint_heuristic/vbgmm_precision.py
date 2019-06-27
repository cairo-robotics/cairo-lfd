import numpy as np
from scipy import linalg
from scipy.special import digamma, logsumexp, gammaln, multigammaln
from sklearn.cluster import KMeans


def compute_precisions_cholesky(covariances):
    # Pulled from sci-kit learn.
    n_components, n_features, _ = covariances.shape
    precisions_chol = np.empty((n_components, n_features, n_features))
    for k, covariance in enumerate(covariances):
        try:
            cov_chol = linalg.cholesky(covariance, lower=True)
        except linalg.LinAlgError:
            raise ValueError(
                "Error. COllapsed samples. Try decreasing number of components or increasing regularization.")
        precisions_chol[k] = linalg.solve_triangular(cov_chol,
                                                     np.eye(n_features),
                                                     lower=True).T
    return precisions_chol

def _compute_log_det_cholesky(matrix_chol, n_features):
    """Compute the log-det of the cholesky decomposition of matrices.

    Parameters
    ----------
    matrix_chol : array-like
        Cholesky decompositions of the matrices.
        'full' : shape of (n_components, n_features, n_features)
        'tied' : shape of (n_features, n_features)
        'diag' : shape of (n_components, n_features)
        'spherical' : shape of (n_components,)

    covariance_type : {'full', 'tied', 'diag', 'spherical'}

    n_features : int
        Number of features.

    Returns
    -------
    log_det_precision_chol : array-like, shape (n_components,)
        The determinant of the precision matrix for each component.
    """

    n_components, _, _ = matrix_chol.shape
    log_det_chol = (np.sum(np.log(
        matrix_chol.reshape(
            n_components, -1)[:, ::n_features + 1]), 1))

    return log_det_chol

def _log_dirichlet_norm(dirichlet_concentration):
    """Compute the log of the Dirichlet distribution normalization term.

    Parameters
    ----------
    dirichlet_concentration : array-like, shape (n_samples,)
        The parameters values of the Dirichlet distribution.

    Returns
    -------
    log_dirichlet_norm : float
        The log normalization of the Dirichlet distribution.
    """
    return (gammaln(np.sum(dirichlet_concentration)) -
            np.sum(gammaln(dirichlet_concentration)))


def _log_wishart_norm(degrees_of_freedom, log_det_precisions_chol, n_features):
    """Compute the log of the Wishart distribution normalization term.

    Parameters
    ----------
    degrees_of_freedom : array-like, shape (n_components,)
        The number of degrees of freedom on the covariance Wishart
        distributions.

    log_det_precision_chol : array-like, shape (n_components,)
         The determinant of the precision matrix for each component.

    n_features : int
        The number of features.

    Return
    ------
    log_wishart_norm : array-like, shape (n_components,)
        The log normalization of the Wishart distribution.
    """
    # To simplify the computation we have removed the np.log(np.pi) term
    return -(degrees_of_freedom * log_det_precisions_chol +
             degrees_of_freedom * n_features * .5 * np.log(2.) +
             np.sum(gammaln(.5 * (degrees_of_freedom -
                                  np.arange(n_features)[:, np.newaxis])), 0))


class VariationalGMM():

    def __init__(self, n_components=3, max_iter=200, tolerance=1e-6, alpha_prior=None, beta_prior=1, dof=None,
                 wishart_matrix_prior=None, weights_prior=None, means_prior=None, covariances_prior=None,
                 regularization=1e-6):
        self.n_components = n_components  # Number of mixture components (K)
        self.max_iter = max_iter  # number of iterations to run iterative update of varational inference
        self.tolerance = tolerance  # Log-likelihood tolerance for terminating EM
        self.alpha_prior = alpha_prior  # Dirichlet parameter for prior of weights. (1, K)
        self.beta_prior = beta_prior  # scaling on precisions matrix.
        self.wishart_matrix_prior = wishart_matrix_prior
        self.dof = dof  # degrees of freedom for Wishart distribution
        self.covariances_prior = covariances_prior  # Initial covariances of mixands.
        self.weights_prior = weights_prior  # Initial weights of mixands
        self.means_prior = means_prior  # Initial means of mixands
        self.regularization = regularization
        self.fitted = False

    def _initialize_parameters(self, X):
        self.n_samples_ = np.shape(X)[0]  # number of samples
        self.n_features_ = np.shape(X)[1]  # number of features
        self.alpha_prior = 1. / self.n_components
        self.alpha_k = np.full([self.n_components, ], self.alpha_prior)  # dirichlet parameters
        self.means_prior = np.mean(X, axis=0)
        self.weights = self.weights_prior if self.weights_prior is not None else np.diag(
            np.random.uniform(0, 1, self.n_components))
        self.beta_k = np.full(self.n_components, self.beta_prior)  # scale of precision matrix.
        self.log_pi = digamma(self.alpha_k) - digamma(np.sum(self.alpha_k))
        self.log_lambda = np.zeros(self.n_components)
        self.dof = self.dof if self.dof is not None else self.n_features_
        self.nu_k = np.full([self.n_components, ], self.dof)
        self.W_k = np.zeros(
            [self.n_components, self.n_features_, self.n_features_])  # scaling matrix of wishart distribution
        self.W_lb = np.zeros(
            [self.n_components, self.n_features_,
             self.n_features_])  # Collects inverse Wishart scaling matricies for use in lowerbound
        self.W_prior = self.wishart_matrix_prior if self.wishart_matrix_prior is not None else np.atleast_2d(
            np.cov(X.T))
        self.W_prior_inv = np.linalg.inv(self.W_prior)  # Inverse of initial wishart component
        self._kmeans_initialize(X)

    def _kmeans_initialize(self, X):
        kmeans = KMeans(n_clusters=self.n_components, n_init=1,
                                   random_state=None).fit(
            X)
        resp = np.zeros((self.n_samples_, self.n_components))
        resp[np.arange(self.n_samples_), kmeans.labels_] = 1
        self.means = kmeans.cluster_centers_.T
        self._initialize(X, resp)

    def _initialize(self, X, resp):
        """Initialization of the mixture parameters.

        Parameters
        ----------
        X : array-like, shape (n_samples, n_features)

        resp : array-like, shape (n_samples, n_components)
        """
        N_k, x_bar_k, S_k = self._estimate_gaussian_statistics(X, resp)
        self._update_dirichlet_prior_parameter(N_k)
        self._update_wishart_prior_parameters(N_k)
        self._update_gaussian_prior_parameters(N_k)
        self._estimate_weights(N_k)
        self._estimate_means(N_k, x_bar_k)
        self._estimate_wishart_matrix(N_k, x_bar_k, S_k)

    def e_step(self, X):
        self._update_expected_log_pi()
        self._update_expected_log_lambda()
        # In the variational E step, the ultiamte goal is to calculate the responsibilities.
        return self.estimate_responsibilities(X)

    def m_step(self, X, resp):
        N_k, x_bar_k, S_k = self._estimate_gaussian_statistics(X, resp)
        self._update_dirichlet_prior_parameter(N_k)
        self._update_wishart_prior_parameters(N_k)
        self._update_gaussian_prior_parameters(N_k)
        self._estimate_weights(N_k)
        self._estimate_means(N_k, x_bar_k)
        self._estimate_wishart_matrix(N_k, x_bar_k, S_k)
        return N_k, x_bar_k, S_k

    def fit(self, X):
        self._initialize_parameters(X)

        self.lower_bounds_ = []
        prior_lb = -np.inf
        for n in range(0, self.max_iter):
            # E-M Step
            self.log_resp, self.resp = self.e_step(X)
            N_k, x_bar_k, S_k = self.m_step(X, self.resp)
            new_lb = self._compute_lower_bound()
            # new_lb = self._calculate_lower_bound(N_k, x_bar_k, S_k)
            # new_lb = self.elbo()

            self.lower_bounds_.append(new_lb)
            if abs(new_lb - prior_lb) <= self.tolerance:
                print("Converged.")
                break
            prior_lb = new_lb
        if abs(new_lb - prior_lb) > self.tolerance:
            print("Algorithm maximum iterations inadequate to achieve convergence according to given tolerance.")
        self.log_resp, self.resp = self.e_step(X)
        self.fitted = True
        return self

    def predict(self, X):
        if self.fitted is True:
            return self.get_weighted_log_probability(X).argmax(axis=1)
        else:
            raise RuntimeError("Model not fitted.")

    def estimate_responsibilities(self, X):
        log_rho_nk = self.get_weighted_log_probability(X)
        Z = logsumexp(log_rho_nk, axis=1)
        log_resp = log_rho_nk - Z[:, np.newaxis]
        resp = np.exp(log_resp)
        return log_resp, resp

    def log_gauss(self, X):
        # Using scikit learns implementation

        # Get the log determinant of the Cholesky decomposed precisions.
        log_det_chol = (np.sum(np.log(
            self.precisions_cholesky_.reshape(
                self.n_components, -1)[:, ::self.n_features_ + 1]), 1))

        # Get the log probability of the gaussian
        #log_prob = np.empty((self.n_samples_, self.n_components))
        log_prob = np.empty((len(X), self.n_components))
        for k, (mu, prec_chol) in enumerate(zip(self.means_, self.precisions_cholesky_)):
            y = np.dot(X, prec_chol) - np.dot(mu, prec_chol)
            #print('hold')
            #print(k)
            #print(y)
            temp = np.sum(np.square(y), axis=1)
            #print(temp)
            #print(temp.shape)
            #print(len(log_prob))
            #print(len(log_prob[0]))
            log_prob[:, k] = np.sum(np.square(y), axis=1)

        return - .5 * (self.n_features_ * np.log(2 * np.pi) + log_prob) + log_det_chol

    def estimate_log_prob(self, X):
        # log rho, see Bishop 10.46
        log_gauss = self.log_gauss(X)
        #print(log_gauss)
        return log_gauss + .5 * (self.log_lambda - self.n_features_ / self.beta_k)

    def get_weighted_log_probability(self, X):
        log_prob = self.estimate_log_prob(X)
        weighted_log_prob = log_prob + self.log_pi
        return weighted_log_prob

    def _estimate_gaussian_statistics(self, X, resp):
        S_k = np.zeros(
            [self.n_components, self.n_features_, self.n_features_])  # estimated covariances of the components
        N_k = np.sum(resp,
                     axis=0) + 10 * np.finfo(resp.dtype).eps  # from Bishop 10.51, sum or responsibilities for each component i.e. number of data samples in each component
        x_bar_k = np.dot(resp.T, X) / N_k[:, np.newaxis]  # Bishop 10.52
        for k in range(0, self.n_components):
            x_cen = X - x_bar_k[k]
            S_k[k] = np.dot(resp[:, k] * x_cen.T, x_cen) / N_k[k]  # Bishop equation 10.53
            S_k[k].flat[::self.n_features_ + 1] += self.regularization
        return N_k, x_bar_k, S_k

    def _update_dirichlet_prior_parameter(self, N_k):
        self.alpha_k = self.alpha_prior + N_k  # from Bishop 10.58

    def _estimate_weights(self, N_k):
        self.weights = (self.alpha_prior + N_k) / (
                self.n_components * self.alpha_prior + self.n_samples_)  # Bishop 10.69
        # self.weights = (self.alpha_prior + N_k)  # scikit learn doesn't divide by anything...why?

    def _update_wishart_prior_parameters(self, N_k):
        self.nu_k = self.dof + N_k  # from Bishop 10.63 and according to sci-kit learn, it shouldn't have the +1

    def _update_gaussian_prior_parameters(self, N_k):
        self.beta_k = self.beta_prior + N_k  # from Bishop 10.60

    def _estimate_means(self, N_k, x_bar_k):

        self.means = (self.beta_prior * self.means_prior + N_k[:, np.newaxis] * x_bar_k) / self.beta_k[:,
                                                                                           np.newaxis]  # from Bishop 10.61
        self.means_ = self.means

    def _estimate_wishart_matrix(self, N_k, x_bar_k, S_k):
        for k in range(0, self.n_components):
            mean_diff = x_bar_k[k] - self.means_prior
            self.W_k[k] = (self.W_prior + N_k[k] * S_k[k] + N_k[k] * self.beta_prior \
                           / self.beta_k[k] * np.outer(mean_diff,
                                                       mean_diff))  # from Bishop 10.62
            self.W_lb[k] = np.linalg.inv(self.W_k[k])
        self.W_k /= self.nu_k[:, np.newaxis, np.newaxis]
        self.covariances_ = self.W_k
        self.precisions_cholesky_ = compute_precisions_cholesky(self.covariances_)

    def _update_expected_log_pi(self):
        self.log_pi = digamma(self.alpha_k) - digamma(np.sum(self.alpha_k))  # from Bishop 10.66

    def _update_expected_log_lambda(self):
        for k in range(0, self.n_components):
            digamma_sum = 0
            for i in range(1, self.n_features_ + 1):
                digamma_sum += digamma((self.nu_k[k] + 1 - i) / 2)
            self.log_lambda[k] = digamma_sum + self.n_features_ * np.log(2) + np.log(np.linalg.det(self.precisions_cholesky_[k])) # from Bishop 10.65


    def logB(self, W, nu):
        n_col = np.shape(W)[1]

        gamma_sum = 0
        for i in range(1, n_col + 1):
            gamma_sum += gammaln(0.5 * (nu + 1 - i))
        # Compute logB function via Bishop B.79
        return (-0.5 * nu * np.log(np.linalg.det(W)) - (0.5 * nu * n_col * np.log(2) + 0.25 * n_col * (n_col - 1) *
                                                        np.log(np.pi) + gamma_sum))

    def _calculate_lower_bound(self, N_k, x_bar_k, S_k):
        # DECREASES
        log_px = 0
        log_pml = 0
        log_pml2 = 0
        log_qml = 0
        for k in range(0, self.n_components):
            # Here we collect all terms that require summations index by the k-th component.
            diff = x_bar_k[k] - self.means[k]
            # see Bishop 10.71; we remove (- self.n_features_ * np.log(2 * np.pi)) since it is an additive constant.
            log_px = log_px + N_k[k] * (
                    self.log_lambda[k] - self.n_features_ / self.beta_k[k] - self.nu_k[k] * np.trace(
                np.dot(S_k[k], self.W_lb[k])) - self.nu_k[k] * np.dot(
                np.dot(diff, self.W_lb[k]), diff)) - self.n_features_ * np.log(2 * np.pi)

            # see Bishop 10.74
            log_pml = log_pml + self.n_features_ * np.log(self.beta_prior / (2 * np.pi)) + self.log_lambda[k] - \
                      (self.n_features_ * self.beta_prior) / self.beta_k[k] - self.beta_prior * self.nu_k[k] * np.dot(
                np.dot(np.transpose(self.means[k] - self.means_prior),
                       self.W_lb[k]), self.means[k] - self.means_prior)

            # see Bishop 10.74
            log_pml2 = log_pml2 + self.nu_k[k] * np.trace(np.dot(self.W_prior_inv, self.W_lb[k]))

            # see Bishop 10.77
            log_qml = log_qml + 0.5 * self.log_lambda[k] + 0.5 * self.n_features_ * np.log(
                self.beta_k[k] / (2 * np.pi)) \
                      - 0.5 * self.n_features_ - (-self.logB(W=self.W_lb[k], nu=self.nu_k[k]) \
                                                  - 0.5 * (self.nu_k[k] - self.n_features_ - 1) * self.log_lambda[
                                                      k] + 0.5 * self.nu_k[
                                                      k] * self.n_features_)

        log_px = 0.5 * log_px  # see Bishop 10.71
        log_pml = 0.5 * log_pml + self.n_components * self.logB(W=self.W_prior, nu=self.dof) + 0.5 * (
                self.dof - self.n_features_ - 1) * np.sum(self.log_lambda) - 0.5 * log_pml2  # see Bishop 10.74
        log_pz = np.sum(np.dot(self.resp, self.log_pi))  # see Bishop 10.72
        log_qz = np.sum(self.resp * self.log_resp)  # 10.75
        log_pp = np.sum((self.alpha_prior - 1) * self.log_pi) + gammaln(np.sum(self.n_components * self.alpha_prior)) - \
                 self.n_components * np.sum(gammaln(self.alpha_prior))  # 10.73

        log_qp = np.sum((self.alpha_k - 1) * self.log_pi) + gammaln(np.sum(self.alpha_k)) - np.sum(
            gammaln(self.alpha_k))  # 10.76

        # Sum all parts to compute lower bound\
        #print(log_px + log_pz + log_pp + log_pml - log_qz - log_qp - log_qml)
        return log_px + log_pz + log_pp + log_pml - log_qz - log_qp - log_qml


    def elbo(self):
        # ELBO: evidence lower bounds in order to test for convergence.
        # DECREASES
        lb = gammaln(np.sum(self.alpha_prior)) - np.sum(gammaln(self.alpha_prior)) \
             - gammaln(np.sum(self.log_lambda)) + np.sum(gammaln(self.log_pi))
        lb -= self.n_samples_ * self.n_features_ / 2. * np.log(2. * np.pi)
        for k in range(0, self.n_components):
            lb += (-(self.dof * self.n_features_ * np.log(2.)) / 2.) \
                  + ((self.nu_k[k] * self.n_features_ * np.log(2.)) / 2.)
            lb += - multigammaln(self.dof / 2., self.n_features_) \
                  + multigammaln(self.nu_k[k] / 2., self.n_features_)
            lb += (self.n_features_ / 2.) * np.log(np.absolute(self.beta_prior)) \
                  - (self.n_features_ / 2.) * np.log(np.absolute(self.beta_k[k]))
            lb += (self.dof / 2.) * np.log(np.linalg.det(self.W_prior)) \
                  - (self.nu_k[k] / 2.) * np.log(np.linalg.det(self.W_lb[k]))
            lb -= np.dot(np.log(self.alpha_k[k]).T, self.alpha_k[k])
        return lb

    def _compute_lower_bound(self):
        """Estimate the lower bound of the model.

        The lower bound on the likelihood (of the training data with respect to
        the model) is used to detect the convergence and has to decrease at
        each iteration.

        Parameters
        ----------
        X : array-like, shape (n_samples, n_features)

        log_resp : array, shape (n_samples, n_components)
            Logarithm of the posterior probabilities (or responsibilities) of
            the point of each sample in X.

        log_prob_norm : float
            Logarithm of the probability of each sample in X.

        Returns
        -------
        lower_bound : float
        """
        # Contrary to the original formula, we have done some simplification
        # and removed all the constant terms.
        n_features, = self.means_prior.shape

        # We removed `.5 * n_features * np.log(self.degrees_of_freedom_)`
        # because the precision matrix is normalized.
        log_det_precisions_chol = (_compute_log_det_cholesky(
            self.precisions_cholesky_, n_features) -
            .5 * n_features * np.log(self.nu_k))


        log_wishart = np.sum(_log_wishart_norm(
            self.nu_k, log_det_precisions_chol, n_features))

        log_norm_weight = _log_dirichlet_norm(self.alpha_k)

        return (-np.sum(np.exp(self.log_resp) * self.log_resp) -
                log_wishart - log_norm_weight -
                0.5 * n_features * np.sum(np.log(self.beta_k)))

