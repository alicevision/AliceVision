# camera

This module provides different camera models.

## Pinhole camera model

A pinhole camera is a simple imaging device that operates based on the principle of light passing through a tiny aperture or *pinhole*. 
In this camera model, there is no lens involved.
Instead, light travels through the pinhole and projects an inverted image of the external scene onto a photosensitive surface, typically positioned opposite the pinhole. This surface serves as the camera's image plane.
Under this camera model, each point in the scene is projected along a ray through the pinhole into a corresponding point on the image plane.

![The pinhole camera model. An oriented central projective camera.](../../../docs/img/pinholeCamera.png)

Formally, the pinhole camera is represented by a projective model, often called pinhole projection, that maps a 3D point $Q$ into its corresponding image point $q$ with the following equation:

$$ q \sim PQ_i = K[R|t]Q $$

$$
\begin{bmatrix}
q_x\\
q_y\\
1\\
\end{bmatrix} \sim
\begin{bmatrix}
f_x &  & c_u \\
& f_y & c_v \\
&  & 1\\
\end{bmatrix}
\begin{bmatrix}
&  &  & t_x \\
& R_{3 \times 3} &  & t_y \\
&  &  & t_z \\
\end{bmatrix}
\begin{bmatrix}
Q_x\\
Q_y\\
Q_z\\
1\\
\end{bmatrix}
$$

where $P$ is the projection matrix, $K$ is the calibration matrix, and $\sim$ means *up to a scale factor* $\lambda = \frac{1}{Q_z}$.

The projection model is characterized by two sets of parameters:

- Intrinsic parameters $f, c_u, c_v$ of the calibration matrix $K$ model the camera properties:

  - $f_x, f_y$: are the focal distances, i.e. the distance between focal and image plane. They are both expressed in pixels and in a true pinhole camera $f_x = f_y$. These are usually used to model flaws in the camera sensor such as non-squared pixels.

  - $c_u, c_v$: the principal point, i.e. the projection of the z-axis of the camera on the image plane. Ideally, it would be the centre of the image.

- Extrinsic parameters $R, t$ represent the components of the rototranslation bringing points of the world reference frame into the camera reference frame:

  - $R_{3\times3}$: the rotation part,
  - $t_{3\times1}$: the translation part. It is the position of the origin of the world coordinate system expressed in camera reference frame. The camera center $C$ in world coordinates, instead, is $C = -R^{-1} t = -R^T t$ (since $R$ is a rotation matrix).

## API

- Pinhole intrinsic

  - `Pinhole_Intrinsic : public IntrinsicBase` 

    - classic pinhole camera (Focal + principal point and image size).

  - `Pinhole_Intrinsic_Radial_K1 : public Pinhole_Intrinsic`

    - classic pinhole camera (Focal + principal point and image size) + radial distortion defined by one factor.
    - can add and remove distortion

  - `Pinhole_Intrinsic_Radial_K3 : public Pinhole_Intrinsic`

    - classic pinhole camera (Focal + principal point and image size) + radial distortion by three factors.
    - can add and remove distortion

  - `Pinhole_Intrinsic_Brown_T2 : public Pinhole_Intrinsic`

    - classic pinhole camera (Focal + principal point and image size) + radial distortion by three factors + tangential distortion by two factors.
    - can add and remove distortion

  - `Pinhole_Intrinsic_Fisheye : public Pinhole_Intrinsic`

    - classic pinhole camera (Focal + principal point and image size) + fish-eye distortion by four factors.
    - can only be applied to a full frame fish-eye (i.e not to hemispherical ones)
    - can add and remove distortion


- Simple pinhole camera models (intrinsic + extrinsic(pose))

```cpp
// Setup a simple pinhole camera at origin
// Pinhole camera P = K[R|t], t = -RC
Mat3 K;
K << 1000, 0, 500,
   0, 1000, 500,
   0, 0, 1;
PinholeCamera cam(K, Mat3::Identity(), Vec3::Zero());
```
