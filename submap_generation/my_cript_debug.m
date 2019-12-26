camera_dir='/home/huxian/drive/file-drive/HUXIAN/ProjectsDatasets/oxford/123/';
files = dir(fullfile(camera_dir,'*.jpg')); 
img_mean=[];
img_std=[];
for i =1:size(files,1)
    img=imread(strcat(camera_dir,files(i).name));
    img_mean=[img_mean mean2(img)];
    img_std=[img_std std2(img)];
end
a=[img_mean;img_std]';
figure('1')
