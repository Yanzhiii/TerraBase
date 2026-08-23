# Stepp_Rellis
Domain adaptation for STEPP to work on Rellis 3D

STEPP Paper - https://github.com/RPL-CS-UCL/STEPP-Code

Rellis 3D dataset - https://github.com/unmannedlab/RELLIS-3D
  -> working on sequence 003, Full-stack Merged data: (15GB)
      link - [Full-stack Merged data: (15GB)](https://drive.google.com/file/d/1glJzgnTYLIB_ar3CgHpc_MBp5AafQpy9/view)
      
( EDIT - moved to direct downloads, i will attack links below )

EDIT:
  i found this is much easier to work with, instead of extracting topics from the bag file, direct download links are better. i will list them here if downloading again is necessary

1. Full Images (11GB) RGB frames for all 5 sequences - link https://drive.google.com/file/d/1F3Leu0H_m6aPVpZITragfreO_SGtL2yV/view
2. Ouster LiDAR Scan Poses files (174MB) - link https://drive.google.com/file/d/1V3PT_NJhA41N7TBLp5AbW31d0ztQDQOX/view
   ( This contains SLAM poses per scan - odometry for pose porjection )
3. Camera Instrinsic (2KB) - link https://drive.google.com/file/d/1NAigZTJYocRSOTfgFBddZYnDsI_CSpwK/view
   ( Contains camerea Instrinsic )
4. Basler Camera to Ouster LiDAR (3KB) - link https://drive.google.com/file/d/19EOqWS9fDUFp4nsBrMCa69xs9LgIlS2e/view
   (Extrinsic - because the poses are in the LiDAR frame, i will transform to camera/device frame)
5. Full Image Annotations ID Format (94MB) - link https://drive.google.com/file/d/16URBUQn_VOGvUqfms-0I8HHKMtjPHsu5/view
   ( I believe this is the ground truth semantic labels which i need to check how dinoV2 works )


I had requested for dinov3 access to Meta.
I got the access and i am working on comparing dinov2 vs dinov3. 
Codes will be uploaded once everything is neat and cleaned up ( now uploaded )

NOTE ON MODELS
DinoV2_DinoV3\Models\compv2_v3 - autoencoder_dinov2 - autoencoder trained on only RELLIS with DinoV2 
DinoV2_DinoV3\Models\compv2_v3 - autoencoder_dinov3 - autoencoder trained on only RELLIS with DinoV3
For the rellis Dataset, download the files in the above link and keep them in the same folder, the code should be able to handle future pose projection without needing to change anything ( except for path to dataset )
DinoV2_DinoV3\Models\Own_Outputs - own_autoencoder_dinov2 / own_autoencoder_dinov3 - Trained on only our data with DinoV2 / DinoV3
( same conditions as to Rellis 3D )

Make sure adjust "path" for individuals files in the code 
