# sfmData

## A generic SfM data container

`SfM_Data` class contains all the data used to describe input of a SfM problem:

* a collection of **View**

  * the used images

* a collection of **camera extrinsic**

  * the camera poses

* a collection of **camera intrinsic**

  * the camera internal projection parameters

* a **structure**

  * the collection of landmark (3D points associated with 2d view observations)

```
struct SfM_Data
{
  /// Considered views
  Views views;

  /// Considered poses (indexed by view.id_pose)
  Poses poses;

  /// Considered camera intrinsics (indexed by view.id_cam)
  Intrinsics intrinsics;

  /// Structure (3D points with their 2D observations)
  Landmarks structure;

  // ...
}
```


### View concept

The view store information related to an image:

* image filename
* id_view (must be unique)
* id_pose
* id_intrinsic
* image size

Note that thanks to the usage of ids we can defined shared poses & shared intrinsics.

View type is **abstract** and provide a way to add new custom View type: i.e. GeoLocatedView (add GPS position, ...)


### Camera Poses concept

The camera pose store a 3D pose that define a camera rotation and position (camera rotation and center).


### Camera Intrinsic concept

Define the parameter of a camera. It can be shared or not.
Intrinsics parameter are **abstract** and provide a way to easily add new custom camera type.


### Structure/Landmarks concept

It defines the structure:

* 3D point with 2D view features observations.
