# ![AliceVision - Photogrammetric Computer Vision Framework](https://github.com/alicevision/AliceVision/raw/develop/docs/logo/AliceVision_banner.png)

[AliceVision](http://alicevision.github.io) is a Photogrammetric Computer Vision Framework which provides a 3D Reconstruction and Camera Tracking algorithms.
AliceVision aims to provide strong software basis with state-of-the-art computer vision algorithms that can be tested, analyzed and reused.
The project is a result of collaboration between academia and industry to provide cutting-edge algorithms with the robustness and the quality required for production usage.

Learn more details about the pipeline and tools based on it on [AliceVision website](http://alicevision.github.io).

See [results of the pipeline on sketchfab](http://sketchfab.com/AliceVision).


## Photogrammetry

Photogrammetry is the science of making measurements from photographs.
It infers the geometry of a scene from a set of unordered photographies or videos.
Photography is the projection of a 3D scene onto a 2D plane, losing depth information.
The goal of photogrammetry is to reverse this process.

See the [presentation of the pipeline steps](http://alicevision.github.io/#photogrammetry).


## License

The project is released under MPLv2, see [**COPYING.md**](COPYING.md).


## Citation

If you use this project for a publication, please cite the [paper](https://hal.archives-ouvertes.fr/hal-03351139):
  ```
  @inproceedings{alicevision2021,
    title={{A}liceVision {M}eshroom: An open-source {3D} reconstruction pipeline},
    author={Carsten Griwodz and Simone Gasparini and Lilian Calvet and Pierre Gurdjos and Fabien Castan and Benoit Maujean and Gregoire De Lillo and Yann Lanthony},
    booktitle={Proceedings of the 12th ACM Multimedia Systems Conference - {MMSys '21}},
    doi = {10.1145/3458305.3478443},
    publisher = {ACM Press},
    year = {2021}
  }
  ```


## Get the project

Get the source code: `git clone --recursive git://github.com/alicevision/AliceVision`

See [**INSTALL.md**](INSTALL.md) to build the project.

Continuous integration status: [![Build Status](https://travis-ci.org/alicevision/AliceVision.png?branch=develop)](https://travis-ci.org/alicevision/AliceVision) [![Coverage Status](https://coveralls.io/repos/github/alicevision/AliceVision/badge.png?branch=develop)](https://coveralls.io/github/alicevision/AliceVision?branch=develop).


## Launch 3D reconstructions

Use [Meshroom](https://github.com/alicevision/meshroom) to launch the AliceVision pipeline.
 - Meshroom provides a User Interface to create 3D reconstructions.
 - Meshroom provides a command line to launch all the steps of the pipeline.
 - Meshroom is written in python and can be used to create your own python scripts to customize the pipeline or create custom automation.

The User Interface of Meshroom relies on Qt and PySide. The Meshroom engine and command line has no dependency to Qt.


## Contact

Use the public mailing-list to ask questions or request features. It is also a good place for informal discussions like sharing results, interesting related technologies or publications:
> [alicevision@googlegroups.com](mailto:alicevision@googlegroups.com)
> [http://groups.google.com/group/alicevision](http://groups.google.com/group/alicevision)

You can also contact the core team privately on: [alicevision-team@googlegroups.com](mailto:alicevision-team@googlegroups.com).


## Contributing

[![CII Best Practices](https://bestpractices.coreinfrastructure.org/projects/2995/badge)](https://bestpractices.coreinfrastructure.org/projects/2995)

Beyond open source interest to foster developments, open source is a way of life. The project has started as a collaborative project and aims to continue. We love to exchange ideas, improve ourselves while making improvements for other people and discover new collaboration opportunities to expand everybody’s horizon.
Contributions are welcome. We integrate all contributions as soon as it is useful for someone, don't create troubles for others and the code quality is good enough for maintainance.

Please have a look at the [project code of conduct](CODE_OF_CONDUCT.md) to provide a friendly, motivating and welcoming environment for all.
Please have a look at the [project contributing guide](CONTRIBUTING.md) to provide an efficient workflow that minimize waste of time for contributors and maintainers as well as maximizing the project quality and efficiency.

Use github Pull Requests to submit contributions:
> [http://github.com/alicevision/AliceVision/issues](http://github.com/alicevision/AliceVision/issues)

Use the public mailing-list to ask questions or request features and use github issues to report bugs:
> [http://github.com/alicevision/AliceVision/pulls](http://github.com/alicevision/AliceVision/pulls)


## Project history

In 2009, CMP research team from CTU started the PhD thesis of Michal Jancosek supervised by Tomas Pajdla. They released windows binaries of their MVS pipeline, called CMPMVS, in 2012.
In 2009, Toulouse INP, INRIA and Duran Duboi started a French ANR project to create a model based Camera Tracking solution based on natural features and a new marker design called CCTag.
In 2010, Mikros Image and IMAGINE research team (a joint research group between Ecole des Ponts ParisTech and Centre Scientifique et Technique du Batiment) started a partnership around Pierre Moulon’s thesis, supervised by Renaud Marlet and Pascal Monasse on the academic side and Benoit Maujean on the industrial side. In 2013, they released an open source SfM pipeline, called openMVG (“Multiple View Geometry”), to provide the basis of a better solution for the creation of visual effects matte-paintings.
In 2015, Simula, Toulouse INP and Mikros Image joined their efforts in the EU project POPART to create a Previz system based on AliceVision.
In 2017, CTU join the team in the EU project LADIO to create a central hub with structured access to all data generated on set based on AliceVision.

See [CONTRIBUTORS.md](CONTRIBUTORS.md) for the full list of contributors. We hope to see you in this list soon!

