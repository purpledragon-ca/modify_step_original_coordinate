import math
import pytest
from step_parser import (
    parse_direction, parse_cartesian_point,
    axis2_placement_to_matrix, mat4_multiply, mat4_inverse_rigid,
    rotation_matrix_to_euler_xyz,
)

SAMPLE = {
    '#12': "CARTESIAN_POINT('',(1.,2.,3.))",
    '#13': "DIRECTION('',(0.,0.,1.))",
    '#14': "DIRECTION('',(1.,0.,0.))",
}


def test_parse_direction():
    assert parse_direction(SAMPLE, '#13') == pytest.approx((0.0, 0.0, 1.0))


def test_parse_cartesian_point():
    assert parse_cartesian_point(SAMPLE, '#12') == pytest.approx((1.0, 2.0, 3.0))


def test_axis2_placement_identity():
    M = axis2_placement_to_matrix((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
    assert [M[r][0] for r in range(3)] == pytest.approx([1.0, 0.0, 0.0])  # X col
    assert [M[r][1] for r in range(3)] == pytest.approx([0.0, 1.0, 0.0])  # Y col
    assert [M[r][2] for r in range(3)] == pytest.approx([0.0, 0.0, 1.0])  # Z col
    assert [M[r][3] for r in range(3)] == pytest.approx([0.0, 0.0, 0.0])  # origin


def test_rotation_matrix_to_euler_xyz_identity():
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    rx, ry, rz = rotation_matrix_to_euler_xyz(R)
    assert rx == pytest.approx(0.0, abs=1e-9)
    assert ry == pytest.approx(0.0, abs=1e-9)
    assert rz == pytest.approx(0.0, abs=1e-9)


def test_rotation_matrix_to_euler_xyz_z90():
    # Rz(90°): [[0,-1,0],[1,0,0],[0,0,1]]
    R = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    rx, ry, rz = rotation_matrix_to_euler_xyz(R)
    assert rx == pytest.approx(0.0, abs=1e-6)
    assert ry == pytest.approx(0.0, abs=1e-6)
    assert rz == pytest.approx(90.0, abs=1e-4)


def test_mat4_multiply_identity():
    I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    A = [[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [0, 0, 0, 1]]
    result = mat4_multiply(A, I)
    for r in range(4):
        assert result[r] == pytest.approx(A[r])


def test_mat4_inverse_rigid():
    M = [[1, 0, 0, 1], [0, 1, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]]
    inv = mat4_inverse_rigid(M)
    assert [inv[r][3] for r in range(3)] == pytest.approx([-1.0, -2.0, -3.0])
    product = mat4_multiply(M, inv)
    I = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    for r in range(4):
        assert product[r] == pytest.approx(I[r])
