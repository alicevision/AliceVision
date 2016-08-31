close all;
clear all;

setPath;

datasetPath = '/home/lilian/data/Features_Repeatability/vgg_oxford_feat_eval/';

allAlgos= { 'popsift-popsift'};%, 'vlfeat' }; %'vlfeat', 'opencv'};%, 'popsift' };%
algoNames= { 'popsift'}; % 'VLFeat'};%, 'OpenCV'}%;, 'popSift'};%

datasetNames = {'bark','bikes','boat','graf','leuven','trees','ubc','wall'}; %'bark','bikes','boat','graf'
for i=1:length(datasetNames)
    allFolders{i} = [ datasetPath datasetNames{i} ]
end

listImages=[ 1 2 3 4 5 6];

save(sprintf('%s/infos.mat', datasetPath), 'allAlgos', 'allFolders', 'listImages','algoNames');

indA = listImages(1);

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
            
            [v_overlap,v_repeatability,v_nb_corespondences,matching_score,nb_matches,twi] = ...
                repeatability(fileA ,fileB, gtFile, imgA, imgB ,0);
            
            v_overlap
            v_repeatability
            v_nb_corespondences
            matching_score
            nb_matches
            save(sprintf('%s/%s/res-%d-%d.mat',folderName{1}, algoName{1}, indA, indB ), 'v_overlap', 'v_repeatability', ...
                'v_nb_corespondences','matching_score', 'nb_matches');
        end
    end
end