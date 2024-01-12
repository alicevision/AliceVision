
### Versioning

Version = MAJOR (>=1 year), MINOR (>= 1 month), PATCH

Version Status = Dev / Release


### Git

Branches
    develop: active development branch
    master: latest release
    vMAJOR.MINOR: release branch

Tags
    vMAJOR.MINOR.PATCH: tag for each release


### Release Process

 - Check if we need to upgrade some dependencies
   - CMake all-in-one
     - CMakeLists.txt in the root folder (use docker/check_dependencies_releases.py)
     - Push updates to https://hub.docker.com/r/alicevision/alicevision-deps/tags
   - Check vcpkg
     - Update https://github.com/alicevision/vcpkg branch: alicevision_master
     - Upload result on gdrive and update INSTALL.md
   - Update INSTALL.md
     - Update minimal versions if needed
     - Update vcpkg install command if needed
 - Source code
   - Create branch from develop: "rcMAJOR.MINOR"
   - Modify version in code, version status to RELEASE (src/aliceVision/version.hpp)
   - Create Release note (using https://github.com/cbentejac/github-generate-release-note)
     - ```
	   ./github-generate-release-note.py -o alicevision -r AliceVision -m MAJOR.MINOR.PATCH --highlights majorFeature feature --label-include bugfix scope:ci,scope:doc,scope:build sensordb -s updated-asc
	   ```
   - PR to develop: "Release MAJOR.MINOR"
 - Build
   - Build docker & push to dockerhub
   - Build windows & update pre-compiled dependencies on gdrive if needed
 - Git
   - Merge "rcMAJOR.MINOR" into "develop"
   - Push "develop" into "master"
   - Create branch: vMAJOR.MINOR
   - Create tag: vMAJOR.MINOR.PATCH
   - Create branch from develop: "startMAJOR.MINOR"
 - Upload binaries on fosshub
 - Fill up Github release note
 - Prepare "develop" for new developments
   - Upgrade MINOR and reset version status to DEV
   - PR to develop: "Start Development MAJOR.MINOR"


### Upgrade a Release with a PATCH version

 - Source code
   - Create branch from rcMAJOR.MINOR: "rcMAJOR.MINOR.PATCH"
   - Cherry-pick specific commits or rebase required PR
   - Modify version in code (src/aliceVision/version.hpp)
   - Update release note
 - Build step
 - Uploads
 - Github release note

