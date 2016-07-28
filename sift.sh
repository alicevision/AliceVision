#! /bin/bash

USAGE="
Example:
./sift.sh {VL,OCV} {vlfeat,opencv} {bark,bikes,boat,graf,leuven,trees,ubc,wall} {1,2,3,4,5,6}

"
if [ $# -eq 0 ]
  then
    echo "$USAGE"; exit 0 ;
fi

case $1 in
 -h) echo "$USAGE"; exit 0 ;;
  h) echo "$USAGE"; exit 0 ;;
help) echo "$USAGE"; exit 0 ;;
esac

echo "mkdir -p /home/lilian/data/Features_Repeatability/vgg_oxford_feat_eval/$3/$2/";
mkdir -p /home/lilian/data/Features_Repeatability/vgg_oxford_feat_eval/$3/$2/;

echo "Running: ";
echo "Linux-x86_64-RELEASE/openMVG_sample_siftPutative -d $1 -i /home/lilian/data/Features_Repeatability/vgg_oxford_feat_eval/$3/img$4.ppm -o /home/lilian/data/Features_Repeatability/vgg_oxford_feat_eval/$3/$2/";

Linux-x86_64-RELEASE/openMVG_sample_siftPutative -d $1 -i /home/lilian/data/Features_Repeatability/vgg_oxford_feat_eval/$3/img$4.ppm -o /home/lilian/data/Features_Repeatability/vgg_oxford_feat_eval/$3/$2/
