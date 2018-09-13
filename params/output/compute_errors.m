clc, clear, close all
addpath('results');
addpath('export_fig');

row5_data = load("20180524-mavic-ugv-soybean-eschikon-row5_AffineGroundTruth.txt");
list = fileread("file_list.txt");
fList = strsplit(list,'\n');

[s_gt5,s_init5,Aff_gt5,t_gt5] = extractFromGTFile(row5_data);
 
counter = 0;
succ_number = 0;

for i = 1:length(fList)-1
   
   found = strfind(fList{i},'row5');
   if isempty(found) || isnan(found) 
       continue
   end
      
   curr_file = load(fList{i});
   [scl,Aff,t,tn,yn,sn] = extractFromFile(curr_file);
      
   s = scaleFromMatrix(Aff);
   Aff = scaleMatrix(Aff,s);
   yn_rad = yn*(3.14/180);
   Rnorm = [cos(yn_rad) -sin(yn_rad) 0; sin(yn_rad) cos(yn_rad) 0; 0 0 1];
   Affn = Rnorm*Aff;
   
   if norm( sn-1 ) < 0.19 || norm( sn-1 ) > 0.21 || yn > 6  || yn < 4
        continue
   end
   
   diff_Aff = Affn\Aff_gt5;
   s_scl = [s(1)*sn(1) s(2)*sn(2)];
   
   angle_err = max( 0.005, computeAngle( diff_Aff ) );
   scale_err = max( 0.005, norm(s_scl-s_gt5(1:2)) );
   transl_err = max( 0.005, norm(t - t_gt5) );
   
   if (abs(angle_err) <= 0.2) && (abs(scale_err) <= 2.5 ) && ( abs(transl_err) <= 0.1 )  
       succ_number = succ_number+1;
   else
       Aff
       Aff_gt5
       angle_err
   end
       
   counter = counter + 1;
   
end

counter

succ_number / counter

function [s,A,t,tn,yn,sn] = extractFromFile(data)

    A = [ data(1) data(2) data(3); 
          data(5) data(6) data(7);
          data(9) data(10) data(11)];

    t = [ data(4) data(8) data(12)];

    s = [ data(13) data(14) ];
    sn = [ data(15) data(16) ];
    tn = [ data(17) data(18) ] ;
    yn = data(19);
   
end

function [s,s_init,A,t] = extractFromGTFile(data)

    A = [ data(1) data(2) data(3); 
          data(5) data(6) data(7);
          data(9) data(10) data(11)];
          
    t = [ data(4) data(8) data(12) ];
    s_init = [data(13) data(14) 1];
    s = scaleFromMatrix(A);
    A = scaleMatrix(A, s);
    
end

function B = scaleMatrix(A,s)
    B = zeros(3,3);
    B(1,1) = A(1,1)/s(1); B(1,2) = A(1,2)/s(2); B(1,3) = A(1,3)/s(3);
    B(2,1) = A(2,1)/s(1); B(2,2) = A(2,2)/s(2); B(2,3) = A(2,3)/s(3);
    B(3,1) = A(3,1)/s(1); B(3,2) = A(3,2)/s(2); B(3,3) = A(3,3)/s(3);
end

function s = scaleFromMatrix(A)
    s = zeros(3,1);
    s(1) = sqrt( A(1,1)*A(1,1) + A(2,1)*A(2,1) + A(3,1)*A(3,1) );
    s(2) = sqrt( A(1,2)*A(1,2) + A(2,2)*A(2,2) + A(3,2)*A(3,2) );
    s(3) = sqrt( A(1,3)*A(1,3) + A(2,3)*A(2,3) + A(3,3)*A(3,3) );
end

function alpha = computeAngle(A)
    alpha = acos( min(1, max(-1, (trace(A) - 1)/2) ) ); 
end