#!/usr/bin/python
def expand_rect_perimeter(thickness, rect_corners):
    # rect_corners should be list of four points starting with upper right point and ordered clockwise
    ur = [rect_corners[0][0] + thickness, rect_corners[0][1] + thickness]
    lr = [rect_corners[1][0] + thickness, rect_corners[1][1] - thickness]
    ll = [rect_corners[2][0] - thickness, rect_corners[2][1] - thickness]
    ul = [rect_corners[3][0] - thickness, rect_corners[3][1] + thickness]
    return [ur, lr, ll, ul]


if __name__ == "__main__":
    print(expand_rect_perimeter(.15, [[0.828470625, 0.06511343749999998], [0.828470625, -0.1448334375], [0.5619293750000001, -0.1448334375], [0.5619293750000001, 0.06511343749999998]]))