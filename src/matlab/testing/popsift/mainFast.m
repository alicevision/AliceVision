close all;
clear all;

% Set additional path
setPath;

allAlgos= { 'opencv', 'vlfeat' }; % options: 'opencv', 'vlfeat', 'popsift'
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
            
            %gtFile = sprintf('%s/H%dto%dp', folderName{1},indA,indB);
            
            [fA, descA] = readSiftFromTextFile(fileA);
            [fB, descB] = readSiftFromTextFile(fileB);
            
            %[fB, descsFinalSetB] = extractSIFT(IB, display_);
            
            figure; subplot(1,2,1); hold on;
            hist(fA(3,:),1:0.5:80);
            subplot(1,2,2); hold on;
            hist(fB(3,:),1:0.5:80);
            
            title(algoName{1});
            
            disp(['# extracted keypoints in' imgA ]);
            nPtsA = size(fA,2)
            disp(['# extracted keypoints in' imgB ]);
            nPtsB = size(fB,2)
            
            % Low version of the matching
            [matches, scores] = vl_ubcmatch(descA, descB);
            
%             descsFinalSetA = descsFinalSetA(:,matches(1,:));
%             descsFinalSetB = descsFinalSetB(:,matches(2,:));
%             
%             diagRescale = sum(descsFinalSetA,2);
%             descsFinalSetA = 512*sqrt(descsFinalSetA*diag(1./diagRescale));
%             
%             diagRescale = sum(descsFinalSetB,2);
%             descsFinalSetB = 512*sqrt(descsFinalSetB*diag(1./diagRescale));
%             
%             
%             theDist = sqrt(sum((descsFinalSetA - descsFinalSetB).*(descsFinalSetA - descsFinalSetB),1));
            
            scores = sqrt(scores);
            disp('Nb putative matches');
            nbPutative = size(matches,2)
            
            if display_
                Icomp = IA;
                shiftX = size(IA,2)+50;
                Icomp(:, (shiftX+1):(shiftX+size(IB,2)), :) = IB;
                
                figure; imagesc(Icomp) ; colormap gray ; hold on;
                
                cc=hsv(size(matches,2));
                
                x1l = fA(1,matches(1,:));
                y1l = fA(2,matches(1,:));
                
                h = vl_plotframe(fA(:,matches(1,:)));
                set(h,'color','g','linewidth',2);
                
                fShifted = fB(:,matches(2,:));
                
                x2l = fShifted(1,:);
                y2l = fShifted(2,:);
                x1 = [ x1l ; y1l ; ones(size(x1l)) ];
                x2 = [ x2l ; y2l ; ones(size(x2l)) ];
                
                fShifted(1,:) = fShifted(1,:) + shiftX;
                x2l = fShifted(1,:);
                
                h = vl_plotframe(fShifted);
                set(h,'color','g','linewidth',2);
                
                l = line( [x1l ; x2l ], [y1l ; y2l], 'LineWidth',2, 'Color', 'r');
                
                %     for iColor=1:size(x1l,2)
                %         iColor
                %         l = line( [x1l(:,iColor) ; x2l(:,iColor) ], [y1l(:,iColor) ; y2l(:,iColor)], 'LineWidth',2, 'Color', cc(iColor,:));
                %     end
            end
            
            if doGeometricValidation
                
                x1l = fA(1,matches(1,:));
                y1l = fA(2,matches(1,:));
                
                x2l = fB(1,matches(2,:));
                y2l = fB(2,matches(2,:));
                x1 = [ x1l ; y1l ; ones(size(x1l)) ];
                x2 = [ x2l ; y2l ; ones(size(x2l)) ];
                
                [H, inliers] = ransacfithomography(x1, x2, 0.1);
                
                x1l = x1l(inliers);
                x2l = x2l(inliers);
                y1l = y1l(inliers);
                y2l = y2l(inliers);
                
                'Inliers'
                nbInliers = size(inliers)
                %size(x1)
                ratioInliers = size(inliers)/size(x1)
                if display_
                    l = line( [x1l ; x2l], [y1l ; y2l], 'LineWidth',1, 'Color', 'g');
                end
                
            end
        end
    end
    algoName{1}
    pause
end

