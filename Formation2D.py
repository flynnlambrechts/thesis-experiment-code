import numpy as np


class Formation2D:
    d = 2
    SCALE = 0.75

    def __init__(self, N, H):
        if H.shape[1] != N:
            raise RuntimeError(
                f"Number of agents in agency matrix does not match provided number of agents '{N}'. Adjacency matrix shape: {np.shape(H)}"
            )

        self.N = N
        self.H = H.astype(int)

        self.p_des = np.zeros([self.N, self.d])

        # place points on the unit circle
        if N == 1:
            self.p_des = np.array([[0, 0]])
        else:
            for i in range(self.N):
                # print(i)
                theta = np.deg2rad(360 / self.N * i)
                self.p_des[i, :] = [np.sin(theta), np.cos(theta)]

        self.p_des = self.SCALE * np.transpose(self.p_des)
        
        self.M = H.shape[0]

    def __str__(self):
        return (
            f"Formation2D(N={self.N}, M={self.M}, d={self.d})\n"
            f"Adjacency matrix H:\n{self.H}\n"
            f"Desired positions (p_des):\n{np.round(self.p_des, 4)}"
        )


point = Formation2D(
    1,
    np.transpose(
        np.array(
            # fmt: off
            [[]]
            # fmt: on
        )
    ),
)


line = Formation2D(
    2,
    np.transpose(
        np.array(
            # fmt: off
            [
                [-1],
                [1],
            ]
            # fmt: on
        )
    ),
)


triangle = Formation2D(
    3,
    np.transpose(
        np.array(
            # fmt: off
            [
                [1, -1, 0],
                [0, 1, -1],
                [-1, 0, 1],
            ]
            # fmt: on
        )
    ),
)

square = Formation2D(
    4,
    np.transpose(
        np.array(
            # fmt: off
            [[-1, 0, 0, 1, -1], [1, -1, 0, 0, 0], [0, 1, -1, 0, 1], [0, 0, 1, -1, 0]]
            # fmt: on
        )
    ),
)


pentagon = Formation2D(
    5,
    np.transpose(
        np.array(
            # fmt: off
            [
                [1, 0, 0, 0, -1, -1, 1],
                [-1, 1, 0, 0, 0, 0, 0],
                [0, -1, 1, 0, 0, 0, -1],
                [0, 0, -1, 1, 0, 1, 0],
                [0, 0, 0, -1, 1, 0, 0],
            ]
            # fmt: on
        )
    ),
)


hexagon = Formation2D(
    6,
    np.transpose(
        np.array(
            # fmt: off
            [
                [ 1,  0,  0,  0,  0, -1, -1, -1],
                [-1,  1,  0,  0,  0,  0,  0,  0],
                [ 0, -1,  1,  0,  0,  0,  0,  1],
                [ 0,  0, -1,  1,  0,  0,  0,  0],
                [ 0,  0,  0, -1,  1,  0,  1,  0],
                [ 0,  0,  0,  0, -1,  1,  0,  0],
            ]
            # fmt: on
        )
    ),
)

heptagon = Formation2D(
    7,
    np.transpose(
        # fmt: off
        np.array([
            [ 1,  0,  0,  0,  0, -1,  0, -1, -1,  0,  0],
            [-1,  1,  0,  0,  0,  0,  0,  0,  0,  1,  0],
            [ 0, -1,  1,  0,  0,  0,  0,  0,  0,  0,  0],
            [ 0,  0, -1,  1,  0,  0,  0,  0,  1, -1,  0],
            [ 0,  0,  0, -1,  1,  0,  0,  1,  0,  0,  1],
            [ 0,  0,  0,  0, -1,  0, -1,  0,  0,  0,  0],
            [ 0,  0,  0,  0,  0,  1,  1,  0,  0,  0, -1],
        ])
        # fmt: on
    ),
)


octagon = Formation2D(
    8,
    np.transpose(
        np.array(
            # fmt: off
            [
                [ 1,  0,  0,  0,  0,  0,  0, -1,  1,  0,  0,  0,  0],
                [-1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0, -1,  0],
                [ 0, -1,  1,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0],
                [ 0,  0, -1,  1,  0,  0,  0,  0,  0, -1,  0,  0,  0],
                [ 0,  0,  0, -1,  1,  0,  0,  0, -1,  0, -1,  0, -1],
                [ 0,  0,  0,  0, -1,  1,  0,  0,  0,  0,  0,  1,  0],
                [ 0,  0,  0,  0,  0, -1,  1,  0,  0,  0,  0,  0,  1],
                [ 0,  0,  0,  0,  0,  0, -1,  1,  0,  1,  0,  0,  0],

            ]
            # fmt: on
        )
    ),
)








formations = [point, line, triangle, square, pentagon, hexagon, heptagon, octagon]


def get_formation(N):
    print("Formation N", N)
    return next((formation for formation in formations if formation.N == N), None)


if __name__ == "__main__":
    print(point)
    print(line)
    print(triangle)
    print(square)
    print(pentagon)
