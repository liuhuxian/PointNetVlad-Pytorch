function generate_submaps_images(base_path)
    %To Use: generate_submaps('/media/deep-three/deep_ssd2/Robotcar/2014-05-19-13-05-38')
  
    %%%%%%%%%%%%Folder Locations%%%%%%%%
    %output folder
    output_folder='/home/huxian/drive/nvme/ProjectsDatasets/oxford/';
    output_folder_name=base_path(find(base_path=='/',1,'last')+1:end);
    output_folder=[output_folder,output_folder_name,'/'];
    mkdir(output_folder);
    %lidar
    base_path= strcat(base_path, '/');
    laser='lms_front';
    laser_dir= strcat(base_path,laser,'/');
    pc_output_folder='pointcloud_10m_5overlap/';
    
    %camera
    camera='stereo';
    camera_dir=strcat(base_path,camera,'/centre/');
    models_dir='/home/huxian/drive/file-drive/HUXIAN/Projects/PycharmProjects/robotcar-dataset-sdk-master/models';
    [~, ~, ~, ~, ~, camera_model] = ReadCameraModel(camera_dir, models_dir);
    img_output_dir='img_10m_5overlap/';
    
    %make pc output folder and img folder
    mkdir(strcat(output_folder,pc_output_folder));
    mkdir(strcat(output_folder,img_output_dir));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%Load extrinsics%%%%%%%%
    extrinsics_dir='/home/huxian/drive/file-drive/HUXIAN/Projects/PycharmProjects/robotcar-dataset-sdk-master/extrinsics/';
    laser_extrinisics = dlmread([extrinsics_dir 'lms_front.txt']);
    ins_extrinsics = dlmread([extrinsics_dir 'ins.txt']);
    camera_extrinsics=dlmread([extrinsics_dir camera '.txt']);

    %将点云投影到相机侧，而不是ins侧
    %G_ins_laser = SE3MatrixFromComponents(ins_extrinsics) \ SE3MatrixFromComponents(laser_extrinisics);
    G_ins_laser = SE3MatrixFromComponents(camera_extrinsics) \ SE3MatrixFromComponents(laser_extrinisics);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%Timestamps%%%%%%%%%%%%%
    laser_timestamps = dlmread(strcat(base_path,laser,'.timestamps'));
    stereo_timestamps=dlmread(strcat(base_path,camera,'.timestamps'));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%Parameters%%%%%%%%%%%%%%%
    to_display=0;
    
    start_chunk=1;
    target_pc_size=2048;

    
    %submap generation
    submap_cover_distance=10.0;
    laser_reading_distance=0.025;
    laser_reading_angle=30;
    dist_start_next_frame=5.0;
    start_saving_img_time=0;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%Set up CSV file%%%%%%%%%%%%
    csv_file_name= 'pointcloud_10m_5overlap.csv';
    fid_locations=fopen(strcat(output_folder,csv_file_name), 'w');
    fprintf(fid_locations,'%s,%s,%s\n','timestamp','northing','easting');
    
    csv_file_name= 'laser_to_camera_timestamps_10m_5overlap.csv';
    camera_to_laser_timestamps=fopen(strcat(output_folder,csv_file_name), 'w');
    fprintf(camera_to_laser_timestamps,'%s,%s\n','laser_timestamp','camera_timestamp');
    
    csv_file_name= 'camera_timestamps.csv';
    camera_timestamps=fopen(strcat(output_folder,csv_file_name), 'w');
    fprintf(camera_timestamps,'%s,%s,%s\n','timestamp','northing','easting');
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for chunk = start_chunk:laser_timestamps(end,2)
        %find readings in chunk
        laser_index_start= find(laser_timestamps(:,2) == chunk, 1, 'first');
        laser_index_end= find(laser_timestamps(:,2) == chunk, 1, 'last');
        
        stereo_index_start= find(stereo_timestamps(:,2) == chunk, 1, 'first');
        stereo_index_end= find(stereo_timestamps(:,2) == chunk, 1, 'last');

        
        l_timestamps=laser_timestamps(laser_index_start:laser_index_end,1);
        s_timestamps=stereo_timestamps(stereo_index_start:stereo_index_end,1);

        disp(strcat('Processing chunk: ',num2str(chunk),' Laser Start Index: ',num2str(laser_index_start),' Laser End Index: ',num2str(laser_index_end)));
        
        %the timestamp range in ins file may be shorter than timestamp
        %range in stereo file and laser file, so cut the stereo and laser
        %stamp
        %过滤前后一段帧
        [ins_start_timestamp,ins_end_timestamp]=get_ins_start_end_stamp(strcat(base_path,'/gps/ins.csv'));
        %%%待会删除这段！！
        %ins_start_timestamp=1424184803249210;
        if (chunk==1)
           %remove first few readings (in car park)
           %i doubt that 5000 is too high ,so i change i
           cut_start=0;
           for i=1:size(laser_timestamps,1)
               if(laser_timestamps(i,1)>=ins_start_timestamp)
                   cut_start=i;
                   break;
               end
           end
           l_timestamps=laser_timestamps(laser_index_start+cut_start:laser_index_end,1);
           
           cut_start=0;
           for i=1:size(stereo_timestamps,1)
               if(stereo_timestamps(i,1)>=ins_start_timestamp)
                   cut_start=i;
                   break;
               end
           end
           s_timestamps=stereo_timestamps(stereo_index_start+cut_start:stereo_index_end,1);
        end

        if (chunk==laser_timestamps(end,2))
           %remove last readings
           cut_end=0;
           for i=size(laser_timestamps,1):-1:1
               if(laser_timestamps(i,1)<=ins_end_timestamp)
                   cut_end=size(laser_timestamps,1)-i;
                   break;
               end
           end
           l_timestamps=laser_timestamps(laser_index_start:laser_index_end-cut_end,1);
           
           cut_end=0;
           for i=size(stereo_timestamps,1):-1:1
               if(stereo_timestamps(i,1)<=ins_end_timestamp)
                   cut_end=size(stereo_timestamps,1)-i;
                   break;
               end
           end
           s_timestamps=stereo_timestamps(stereo_index_start:stereo_index_end-cut_end,1);
        end

        %%%%%%%%%%POSES%%%%%%%%%%
        laser_global_poses=getGlobalPoses(strcat(base_path,'/gps/ins.csv'), l_timestamps');
        camera_global_poses=getGlobalPoses(strcat(base_path,'/gps/ins.csv'), s_timestamps');
        disp(strcat('Processing chunk: ',num2str(chunk),' Loaded laser poses'));
        %%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%Counter Variables%%%%
        %laser
        frame_start=1;
        frame_end=frame_start+1;
        frames=[];
        i=frame_start;
        j=i;
        start_next_frame=frame_start;

        got_next=0;
        
        start_stereo_idx=1;
        %%%%%%%%%%%%%%%%%%%%%%%%%%

        while(frame_end<length(l_timestamps))
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%GET SCANS TO GENERATE SUBMAP%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            while(getDistance(laser_global_poses{i}(1,4), laser_global_poses{i}(2,4),laser_global_poses{frame_start}(1,4), laser_global_poses{frame_start}(2,4))<submap_cover_distance)
                if(j>(length(l_timestamps)-1))
                   break
                end  
                j=j+1;  

                while((getDistance(laser_global_poses{i}(1,4), laser_global_poses{i}(2,4), laser_global_poses{j}(1,4), laser_global_poses{j}(2,4))<laser_reading_distance)...
                       && (getRotation(laser_global_poses{i}(1:3,1:3), laser_global_poses{j}(1:3,1:3))*180/pi <laser_reading_angle))
                    j=j+1;
                    if(j>(length(l_timestamps)-1))
                        break
                    end  
                end
                frames=[frames j];

                if(j>(length(l_timestamps)-1))
                    break
                end

                if(getDistance(laser_global_poses{frame_start}(1,4), laser_global_poses{frame_start}(2,4), laser_global_poses{j}(1,4), laser_global_poses{j}(2,4))>dist_start_next_frame && got_next==0)
                  start_next_frame=frames(1,end);
                  got_next=1;
                end
            i=j;
            end

            if(j>length(l_timestamps)-1)
                break
            end  
            frame_end=j;        
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %%%%%%%Build Pointcloud%%%%%%%
            pointcloud = [];
            for i=frames
                scan_path = [laser_dir num2str(l_timestamps(i,1)) '.bin'];
                scan_file = fopen(scan_path);
                scan = fread(scan_file, 'double');
                fclose(scan_file);

                scan = reshape(scan, [3 numel(scan)/3]);
                scan(3,:) = zeros(1, size(scan,2));

                scan = inv(laser_global_poses{frame_start})*laser_global_poses{i} * G_ins_laser * [scan; ones(1, size(scan,2))];
                pointcloud = [pointcloud scan(1:3,:)];
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            %%%Remove ground plane
            [normal, in_plane, out_plane]=pcfitplane(pointCloud(pointcloud'),0.5);

            %%%%%%%%%%%Check if not enough points after road removal
            if (size(out_plane,1)<target_pc_size)
                %reset variables
                if (got_next==0)
                   frame_start=frame_start+50;
                   start_next_frame=frame_start+7;
                else
                   frame_start=start_next_frame;
                   start_next_frame=frame_start;
                end
                frame_end= frame_start+1;
                frames=[frame_start];
                i=frame_start;
                j=i;               
                got_next=0;

                disp('Faulty pointcloud');
                continue 
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %%%%%%%%Downsample to exactly target_pc_size points%%%%%%%%%%%%
            out_of_plane=pointcloud(:,out_plane);
            
            %find appropriate scale
            scale_size=1.001;
            downsampled=pcdownsample(pointCloud(out_of_plane'),'gridAverage',scale_size);
            
            while (downsampled.Count()<target_pc_size)
               scale_size=scale_size-0.025;
               if(scale_size<=0)
                    xyz=out_of_plane';
                    break;
               end
               downsampled=pcdownsample(pointCloud(out_of_plane'),'gridAverage',scale_size);
            end
            
            while (downsampled.Count()>target_pc_size)
               scale_size=scale_size+0.025;
               downsampled=pcdownsample(pointCloud(out_of_plane'),'gridAverage',scale_size);
            end
            
            if(scale_size>0)
                xyz=[downsampled.Location(:,1),downsampled.Location(:,2),downsampled.Location(:,3)];
            end 
            
            %add additional random points
            num_extra_points=target_pc_size-size(xyz,1);
            permutation=randperm(length(out_of_plane));
            sample_out=permutation(1:num_extra_points);
            sample=out_of_plane(:,sample_out);%3xn            
            
            output=[xyz',sample];            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %%%%%%%%%%%%%%%%%Transform pointcloud%%%%%%%%%%%%%%%%%%%%%%%%%%
            %transform wrt the centroid
            x_cen=mean(output(1,:));
            y_cen=mean(output(2,:));
            z_cen=mean(output(3,:));
            centroid=[x_cen;y_cen;z_cen;1];
            centroid_g=double(laser_global_poses{frame_start})*double(centroid);
            
            %make spread s=0.5/d
            num_sum=0;
            for i=1:size(output,2)
                num_sum=num_sum+sqrt((output(1,i)-x_cen)^2+(output(2,i)-y_cen)^2+(output(3,i)-z_cen)^2);
            end
            d=num_sum/size(output,2);
            s=0.5/d;

            T=[[s,0,0,-s*(x_cen)];...
            [0,s,0,-s*(y_cen)];...
            [0,0,s,-s*(z_cen)];...
            [0,0,0,1]];
            scaled_output=T*[output; ones(1, size(output,2))];
            scaled_output=-scaled_output;
            
            %Enforce to be in [-1,1] and have exactly target_pc_size points
            cleaned=[];
            for i=1:size(scaled_output,2)
               if(scaled_output(1,i)>=-1 && scaled_output(1,i)<=1 && scaled_output(2,i)>=-1 && scaled_output(2,i)<=1 ...
                       && scaled_output(3,i)>=-1 && scaled_output(3,i)<=1)
                    cleaned=[cleaned,scaled_output(:,i)];
               end
            end
            
            %make number of points equal to target_pc_size
            num_extra_points=target_pc_size-size(cleaned,2);
            disp(strcat(num2str(size(cleaned,2)),'.',num2str(num_extra_points)));
            permutation=randperm(length(out_of_plane));
            i=1;
            while size(cleaned,2)<target_pc_size
               new_point=-T*[out_of_plane(:,permutation(1,i));1];
               if(new_point(1,1)>=-1 && new_point(1,1)<=1 && new_point(2,1)>=-1 && new_point(2,1)<=1 ...
                       && new_point(3,1)>=-1 && new_point(3,1)<=1)                
                    cleaned=[cleaned,new_point];
               end
               i=i+1;
            end
            cleaned=cleaned(1:3,:);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %%%Double check
            if(size(cleaned,2)~=target_pc_size)
               frame_start=start_next_frame;
               frame_end= frame_start+1;
               frames=[frame_start];
               i=frame_start;
               j=i;
               disp('Invalid pointcloud')
               continue;
            end
            
            %%%Output Files
            %output pointcloud in binary file
            origin_timestamp=l_timestamps(frames(1,1),1);
            end_timstamp=l_timestamps(frames(1,end),1);
            fileID = fopen(strcat(output_folder,pc_output_folder, num2str(origin_timestamp),'.bin'),'w');
            fwrite(fileID,cleaned,'double');
            fclose(fileID);
            disp(num2str(origin_timestamp));
            
            %output images
            time_idxes=[];
            
            for time_idx=start_stereo_idx:size(s_timestamps,1)
                time=s_timestamps(time_idx,1);
                if(time<origin_timestamp)
                    start_stereo_idx=time_idx;
                elseif(time>=origin_timestamp&&time<end_timstamp)
                    time_idxes=[time_idxes time_idx];
                else
                    break
                end        
            end
            
            for img_timestamp_idx=time_idxes
                img_timestamp=s_timestamps(img_timestamp_idx);
                img=LoadImage(camera_dir,img_timestamp,camera_model);
                
                %if image is overexposure ,continue next
                if(mean2(img)>210)
                    continue
                end
                 
                if(img_timestamp>start_saving_img_time)
                    img=imresize(img,[480,640]);
                    imwrite(img,strcat(output_folder,img_output_dir, num2str(img_timestamp),'.jpg'));
                    fprintf(camera_timestamps, '%s,%s,%s\n',num2str(img_timestamp),...
                    camera_global_poses{img_timestamp_idx}(1,4),camera_global_poses{img_timestamp_idx}(2,4));
                end
                
                fprintf(camera_to_laser_timestamps, '%s,%s\n',num2str(origin_timestamp),num2str(img_timestamp));    
            end
            start_saving_img_time=s_timestamps(time_idxes(1,end));
            
            %write line in csv file and out put images
            fprintf(fid_locations, '%s,%f,%f\n',num2str(origin_timestamp),centroid_g(1,1), centroid_g(2,1));

            
            
            %%%Display
            if(to_display)
                figure(1);
                pcshow(cleaned');
                axis equal;
                pause
            end
            
            %%%%%%%Reset Variables%%%%%%
            if (got_next==0)
               frame_start=frame_start+50;
            else
               frame_start=start_next_frame;
            end
            frame_end= frame_start+1;
            frames=[frame_start];
            i=frame_start;
            j=i;               
            got_next=0;    
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        end
    end
    
    fclose(fid_locations);
    fclose(camera_to_laser_timestamps);
    plot_pointcloud_path(output_folder);
end
