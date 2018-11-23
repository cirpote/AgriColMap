addpath('20180524-mavic-ugv-soybean-eschikon-row3/CPD_Comparison');

GroundTruth = load("20180524-mavic-ugv-soybean-eschikon-row3_AffineGroundTruth.txt");
AffesultsList = fileread("row3_CPD_list.txt");
Files_List = strsplit(AffesultsList,'\n');

Result = [];

for i = 1:length(Files_List)

    curr_file = load(Files_List{i});

    %Affine Matrix
    Aff = [ curr_file(1) curr_file(2) curr_file(3); 
          curr_file(5) curr_file(6) curr_file(7);
          curr_file(9) curr_file(10) curr_file(11)];

    t = [ curr_file(4) curr_file(8) curr_file(12)];

    xy_err = [ curr_file(15) curr_file(16)];
    yaw_err = curr_file(18);
    xy_scl_err = [curr_file(19)-1 curr_file(20)-1];



    scl_x = sqrt(Aff(1,1)*Aff(1,1) + Aff(2,1)*Aff(2,1) + Aff(3,1)*Aff(3,1));
    scl_y = sqrt(Aff(1,2)*Aff(1,2) + Aff(2,2)*Aff(2,2) + Aff(3,2)*Aff(3,2));
    scl_z = sqrt(Aff(1,3)*Aff(1,3) + Aff(2,3)*Aff(2,3) + Aff(3,3)*Aff(3,3));

    R = [ Aff(1,1)/scl_x Aff(1,2)/scl_y Aff(1,3)/scl_z;
          Aff(2,1)/scl_x Aff(2,2)/scl_y Aff(2,3)/scl_z;
          Aff(3,1)/scl_x Aff(3,2)/scl_y Aff(3,3)/scl_z;];

    tr = R(1,1) + R(2,2) + R(3,3);  

    % Affotational Error
    rot_err = acos( min( 1, max(-1, (tr -1)/2) ) ); 
    %Translational Error
    trans_err = sqrt(t(1)*t(1) + t(2)*t(2));
    %Scale Error
    scl_err = sqrt(scl_x*scl_x + scl_y*scl_y);
    
    xy_err_mag = sqrt(xy_err(1)*xy_err(1) + xy_err(2)*xy_err(2));
    xy_scl_err_mag = sqrt( xy_scl_err(1)*xy_scl_err(1) + xy_scl_err(2)*xy_scl_err(2) );
    
    Result = [Result; rot_err, trans_err, scl_err, xy_err_mag, yaw_err, xy_scl_err_mag];
end
