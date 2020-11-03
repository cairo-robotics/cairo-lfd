
import os
import pprint

from cairo_lfd.constraints.concept_constraints import ConstraintFactory
from cairo_lfd.middleware.ar_middleware import remap_constraints_for_lfd
from cairo_lfd.data.io import load_json_file

if __name__ == "__main__":
    fd = os.path.dirname(os.path.abspath(__file__))
    ar_constraint_data = load_json_file(fd + "/ar_constraints.json")
    new_constraint_configs = remap_constraints_for_lfd(ar_constraint_data)
    constraints = ConstraintFactory(new_constraint_configs).generate_constraints()
    for constraint in constraints:
        print(constraint)