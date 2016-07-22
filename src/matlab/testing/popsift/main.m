close all;
clear all;

setPath;

allAlgos= { 'opencv', 'vlfeat' }; %'opencv', 'vlfeat', 'popsift'
datasetPath = '/home/lilian/data/Features_Repeatability/vgg_oxford_feat_eval/';

datasetNames = { 'boat' };
for i=1:length(datasetNames)
    allFolders{i} = [ datasetPath datasetNames{i} ]
end

listImages=[1 4];

indA = listImages(1);
display_ = 0;

doGeometricValidation = 1;

for algoName = allAlgos
    for folderName=allFolders
        for indB = listImages(indA+1:end)
            indB
            disp(['Results for ' algoName{1}]);
            
            fileA = sprintf('%s/%s/img%d.sift', folderName{1}, algoName{1}, indA)
            fileB = sprintf('%s/%s/img%d.sift', folderName{1}, algoName{1}, indB)
            
            imgA = sprintf('%s/img%d.ppm', folderName{1}, indA)
            imgB = sprintf('%s/img%d.ppm', folderName{1}, indB)
            
            gtFile = sprintf('%s/H%dto%dp', folderName{1},indA,indB)
            
            [v_overlap,v_repeatability,v_nb_of_corespondences,matching_score,nb_of_matches,twi] = ...
                repeatability(fileA ,fileB, gtFile, imgA, imgB ,0);
            
            pause
        end
    end
end