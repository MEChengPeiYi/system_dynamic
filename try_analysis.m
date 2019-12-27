%written by: Jui-Wen, Yeh 
%For PePe's second lab
%try to analyze the picture, distinguish colors from it
clc; clear all
pic = imread ('SharedScreenshot.jpg'); 
figure (1)
% imshow(pic)
gray_pic = rgb2gray(pic); 
[size_pic_x, size_pic_y] = size (gray_pic); 
R_shell_pic = uint8(pic(:, :, 1));
G_shell_pic = uint8(pic(:, :, 2));
B_shell_pic = uint8(pic(:, :, 3)); 

figure (2)
cut_x = 10; 
cut_y = 95; 
cropped = imcrop(pic, [cut_y cut_x size_pic_y-cut_y size_pic_x-cut_x]);
imshow(cropped)

% test out substract method
R = imsubtract(R_shell_pic, gray_pic); 
figure (200)
R = im2bw(R, 0.30);
imshow(R)

%do the video processing 
video_1 = VideoReader('1-1-90--90.mov'); 
get (video_1) 

%for frame_by_frame
mkdir Frame_by_frame

%cut video into pcitures and substract red dot
NumberOfFrames =uint8( video_1.Duration .* video_1.FrameRate); 
video_tear_down = zeros(video_1.Height, video_1.Width, NumberOfFrames);
video_tear_down_process = video_tear_down; 
centroid_point = zeros(NumberOfFrames, 2);

for img = 1:NumberOfFrames
   
   t=img/NumberOfFrames;
    process  = read(video_1, img); 
    %process to find the red dot in the image
    gray_process = rgb2gray(process);
    R_shell_process = uint8(process(:, :, 1));    
    R_process = imsubtract(R_shell_process, gray_process); 
    R_process = im2bw(R_process, 0.34);
    video_tear_down_process(:, :, img) = R_process;
    cut_x = 30; 
cut_y =130; 
cropped = imcrop(R_process, [cut_y cut_x size_pic_y-cut_y size_pic_x-cut_x]);


    %find the centroid of the figure
    R_process = imfill(R_process,'holes');
    se = strel('line',5,5);
    R_process = imerode(R_process,se);
    L = bwlabel(R_process,8);
   s = regionprops(R_process,'centroid');
   theta10=[];
   theta20=[];
    hold on;
    
     centroids = cat(1,s.Centroid);  
     Csize=size(centroids);
     dotnumber=Csize(1);
     figure(4);
     if (dotnumber==3 )
         x1=centroids(1,1);x2=centroids(2,1);x3=centroids(3,1);
         y1=centroids(1,2);y2=centroids(2,2);y3=centroids(3,2);
         theta1=atan((x2-x1)/(y2-y1));
         theta2=atan((x3-x2)/(y3-y2))+theta1; 
         
         plot(img,theta1,'ro--');
         plot(img,theta2,'bo--');
     elseif (dotnumber==4)
         x1=centroids(1,1);
         x2=(centroids(2,1)+centroids(3,1))/2;
         x3=centroids(4,1);
         y1=centroids(1,2);
         y2=(centroids(2,2)+centroids(3,2))/2;;
         y3=centroids(4,2);
         theta1=atan((x2-x1)/(y2-y1));
         theta2=atan((x3-x2)/(y3-y2))+theta1; 
        
          plot(img,theta1,'ro');
          plot(img,theta2,'bo');
  
     end
    i=i+1;
    %save file 
    hold off
    cd Frame_by_frame
    filename=strcat('frame',num2str(img),'.jpg');
    imwrite(R_process,filename);
    cd ..
    
   
   
    

    
end 
