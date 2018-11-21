import cv2
import numpy as np



def rot2quat(R):

    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = R[:3,:3].flat
    # Fill only lower half of symmetric matrix
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0              ],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0              ],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0              ],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
        ) / 3.0
    # Use Hermitian eigenvectors, values for speed
    vals, vecs = np.linalg.eigh(K)
    # Select largest eigenvector, reorder to x,y,z, w quaternion
    q = vecs[[0, 1, 2, 3], np.argmax(vals)]
    # Prefer quaternion with positive w
    # (q * -1 corresponds to same rotation as q)
    if q[0] < 0:
        q *= -1
    return q


def yml2transforms(filename, size):

    fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)

    pattern = "T%d_%d"
    hand_transforms = []
    camera_transforms = []
    for j in range(size):

        for i in range(1,3):
            node = fs.getNode(pattern%(i,j))

            transform = np.asarray(node.mat())
            if i == 1:
                hand_transforms.append(transform)
            else:
                camera_transforms.append(transform)

            # q = rot2quat(transform[:3,:3])
            # p = transform[:3,3]

            # print p, q

    
    return hand_transforms, camera_transforms


