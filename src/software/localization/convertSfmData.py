#!/usr/bin/env python
# -*- coding: utf-8 -*-

# sfm_data changelog
#
# v0.1.0
#   original version
# v0.2.0
#   added control points (optional)
#   Views: polymorphic views
#   Structure: added rgb
# v0.3.0
#   added featureFolder and matchingFolder (both optional)
#   Intrinsics: added serialNumber and initialFocalLengthPix
#   Structure: added descType

import json
import argparse
import os


def create_symlink_new_file_name(view_filename, feature_folder, id_view, desc_type, extension):
    """
    It fixes the naming convention for the .desc and .feat files. The current convention is
    id_view.descType.{desc,feat}.
    It will search the feature_folder for such a file, if it is not present it will generate a
    symlink with this name for previous naming convention:
    imageName.{feat,desc}
    or
    id_view.{feat,desc}

    :param view_filename: The filename with extension of the image of the view (e.g. DSC_5432.jpeg)
    :param feature_folder: The path of the folder containing the features files
    :param id_view: The ID of the current view
    :param desc_type: The type of descriptor
    :param extension: either '.desc' or '.feat'
    :return True if a symlink is generated, false if nothing has been done
    """
    name, img_ext = os.path.splitext(os.path.basename(view_filename))
    # file name based on image filename (very old style)
    path_based_on_imagename = os.path.join(feature_folder, name + extension)
    # file name based on view ID without the type of descriptor
    path_based_on_viewid = os.path.join(feature_folder, str(id_view) + extension)
    # standard filename with view ID and type of descriptor
    path_to_use = os.path.join(feature_folder, str(id_view) + '.' + desc_type + extension)

    # if the correct file does not exist already
    if not os.path.isfile(path_to_use):
        # if there is the one based on viewid
        if os.path.isfile(path_based_on_viewid):
            os.symlink(path_based_on_viewid, path_to_use)
            print('Generated symlink ' + path_to_use + ' for View ' + str(id_view))
            return True
        # if there is the one based on image filename
        elif os.path.isfile(path_based_on_imagename):
            os.symlink(path_based_on_imagename, path_to_use)
            print('Generated symlink ' + path_to_use + ' for View ' + str(id_view))
            return True
        else:
            print('WARNING: no ' + extension + ' file can be found for View ' + str(id_view))
            return False
    return False



descriptorTypes = {
    'UNKNOWN': 0,
    'UNINITIALIZED': 1,
    'SIFT': 10,
    'SIFT_FLOAT': 11,
    'AKAZE': 20,
    'AKAZE_LIOP': 21,
    'AKAZE_MLDB': 22,
    'CCTAG3': 30,
    'CCTAG4': 31,
    'SIFT_OCV': 40,
    'AKAZE_OCV': 41
}

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Update an older version of sfm_data to the most recent one')
    parser.add_argument('-i', '--inputsfm', required=True, help='The sfm data file to convert')
    parser.add_argument('-o', '--outputsfm', required=True, help='The name of the converted sfm data file')

    # Root parameters
    rootParameters = parser.add_argument_group('Root', 'Values to be filled at root level of the file.')
    rootParameters.add_argument('--featureFolder', default='',
                                help='The folder containing the features (default: %(default)s)')
    rootParameters.add_argument('--matchingFolder', default='',
                                help='The folder containing the matches (default: %(default)s)')

    # Structure parameters
    structureParameters = parser.add_argument_group('Structure', 'Values to be filled inside structure (Landmarks) for '
                                                                 'missing fields. If the field already exist, '
                                                                 'the argument is ignored.')
    structureParameters.add_argument('--descType', default='SIFT', choices=list(descriptorTypes.keys()),
                                     help='The type of features (default: %(default)s)')
    structureParameters.add_argument('--color', default=[255, 255, 255], metavar='[0...255]', nargs=3,
                                     help='The default color to assign to each point. Enter three integer in the '
                                          'range [0...255] for the rgb color value (default: %(default)s)')

    # fix features file names
    filenamesParameters = parser.add_argument_group('Filenames fixer',
                                                    'Update the filename .desc and .feat to latest convention.')
    filenamesParameters.add_argument('--fixFilenames', action='store_true',
                                     help='Try to fix the filenames for descriptors (.desc) and filenames (.feat)')

    args = parser.parse_args()

    defaultColor = map(int, args.color)

    with open(args.inputsfm) as data_file:
        data = json.load(data_file)

    ##########################
    # file version
    print('Input sfm data file version ' + data['sfm_data_version'])
    print('Updating to version 0.3.0')
    data['sfm_data_version'] = "0.3.0"

    ##########################
    # root elements
    if data.get('featureFolder') is None:
        if not args.featureFolder:
            print("Warning: no path is given for the feature folder, the resulting file may be working incorrectly")
        data['featureFolder'] = args.featureFolder
        print('featureFolder set to ' + data['featureFolder'])

    if data.get('matchingFolder') is None:
        if not args.matchingFolder:
            print("Warning: no path is given for the matches folder, the resulting file may be working incorrectly")
        data['matchingFolder'] = args.matchingFolder
        print('matchingFolder set to ' + data['matchingFolder'])

    ##########################
    # updating the views
    print('\nUpdating Views')
    numView = len(data['views'])
    print('Found ' + str(numView) + ' Views')

    viewChanged = 0
    for i in range(numView):
        if data['views'][i]['value'].get("polymorphic_id") is None:
            data['views'][i]['value']["polymorphic_id"] = 1073741824
            viewChanged += 1
    if viewChanged > 0:
        print('updated ' + str(viewChanged) + ' views')
    else:
        print('no changes needed for views')

    ##########################
    # updating the Intrinsics
    print('\nUpdating Intrinsics')
    numIntrinsics = len(data['intrinsics'])
    print('Found ' + str(numIntrinsics) + ' Intrinsics')

    serialChanged = 0
    initialChanged = 0
    for i in range(numIntrinsics):
        if data['intrinsics'][i]['value']['ptr_wrapper']['data'].get("serialNumber") is None:
            data['intrinsics'][i]['value']['ptr_wrapper']['data']["serialNumber"] = ""
            serialChanged += 1
        if data['intrinsics'][i]['value']['ptr_wrapper']['data'].get("initialFocalLengthPix") is None:
            data['intrinsics'][i]['value']['ptr_wrapper']['data']["initialFocalLengthPix"] = -1
            initialChanged += 1
    if viewChanged > 0:
        print('inserted ' + str(serialChanged) + ' serialNumber')
    if initialChanged > 0:
        print('inserted ' + str(initialChanged) + ' initialFocalLengthPix')
    if viewChanged == 0 and initialChanged == 0:
        print('no changes needed for intrinsics')

    ##########################
    # updating the Structure
    print('\nUpdating Structure')
    numLandmarks = len(data['structure'])

    print('Found ' + str(numLandmarks) + ' Landmarks')

    rgbChanged = 0
    descChanged = 0
    for i in range(numLandmarks):
        if data['structure'][i]['value'].get("rgb") is None:
            data['structure'][i]['value']["rgb"] = defaultColor
            rgbChanged += 1
        if data['structure'][i]['value'].get("descType") is None:
            data['structure'][i]['value']["descType"] = descriptorTypes[args.descType]
            descChanged += 1

    if rgbChanged == 0 and descChanged == 0:
        print('no changes needed for structure')
    else:
        print('inserted ' + str(rgbChanged) + ' rgb')
        print('inserted ' + str(descChanged) + ' descType')

    ##########################
    # if there is no control points add empty list
    if data.get('control_points') is None:
        data['control_points'] = []
        print('inserted an empty control points list')

    ##########################
    # writing the updated version
    print('\nWriting file...')
    with open(args.outputsfm, 'w') as outfile:
        json.dump(data, outfile, indent=2)
    print('File updated successfully')

    ##########################
    if args.fixFilenames and (args.featureFolder != ''):
        print('\nFixing the filenames of the features...')

        descGenerated = 0
        featGenerated = 0
        for v in data['views']:
            idView = v['value']['ptr_wrapper']['data']['id_view']
            filename, ext = os.path.splitext(os.path.basename(v['value']['ptr_wrapper']['data']['filename']))

            if create_symlink_new_file_name(view_filename=filename,
                                            feature_folder=args.featureFolder,
                                            id_view=idView,
                                            desc_type=args.descType,
                                            extension='.desc'):
                descGenerated += 1

            if create_symlink_new_file_name(view_filename=filename,
                                            feature_folder=args.featureFolder,
                                            id_view=idView,
                                            desc_type=args.descType,
                                            extension='.feat'):
                featGenerated += 1

        if featGenerated == 0 and descGenerated == 0:
            print('no symlink generated')
        else:
            print('generated ' + str(featGenerated) + ' symlink for .feat files')
            print('generated ' + str(descGenerated) + ' symlink for .desc files')
