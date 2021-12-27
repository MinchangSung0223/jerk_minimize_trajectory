function vector = TransToEulerp(T)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes the transformation matrix T in SE(3) 
% Returns R: the corresponding rotation matrix
%         p: the corresponding position vector .
% Example Input:
% 
% clear; clc;
% T = [[1, 0, 0, 0]; [0, 0, -1, 0]; [0, 1, 0, 3]; [0, 0, 0, 1]];
% [R, p] = TransToRp(T)
% 
% Output:
% R =
%     1     0     0
%     0     0    -1
%     0     1     0
% p =
%     0
%     0
%     3

vector=se3ToVec(MatrixLog6(T))
end