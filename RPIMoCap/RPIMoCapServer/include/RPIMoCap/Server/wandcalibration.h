/*
 * This file is part of the RPIMoCap (https://github.com/kaajo/RPIMoCap).
 * Copyright (c) 2019 Miroslav Krajicek.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <RPIMoCap/Core/cameraparams.h>
#include <RPIMoCap/Server/camerasettings.h>
#include <RPIMoCap/Core/msgpack_defs.h>

#include <QObject>
#include <QMap>

#include <opencv2/core/mat.hpp>

class WandCalibration : public QObject
{
    Q_OBJECT
public:
    WandCalibration(QMap<int,std::shared_ptr<CameraSettings>> &cameraSettings,
                    RPIMoCap::CameraParams camData, QObject *parent = nullptr);

    void addFrame(const QMap<int, std::vector<cv::Point2f> > &points);

private:
    std::optional<Eigen::Affine3f> extrinsicGuess(const size_t camID, const std::vector<cv::Point2f> &pixels);
    static std::vector<cv::Point2f> detect4pWand(const std::vector<cv::Point2f> &pts);

    std::vector<cv::Point3f> m_wandPoints;

    QMap<int,std::shared_ptr<CameraSettings>> m_cameraSettings;
    QMap<int, std::optional<Eigen::Affine3f>> m_extrinsicGuess;
    bool haveAllExtrinsicGuess();

    RPIMoCap::CameraParams m_camData;

    std::vector<QMap<int, std::vector<cv::Point2f>>> m_observations;
};

/*

function [c,xyzMat,R,tv,sf,xyzMat2,ptMatu,ptMat2u,f,UoVo,nlin]=sbaCalib(ptMat,
 UoVo,wandLen,ptMat2,f,nlin,optimMode,distortionMode)

% function
% [c,xyzMat,R,tv,sf,xyzMat2,ptMatu,ptMat2u,f,UoVo,nlin]=sbaCalib(ptMat...
%    UoVo,wandLen,ptMat2,f,nlin,optimMode,distortionMode)
% Camera calibration function to estimate camera extrinsics and intrinsics
%   for C cameras from a set of shared wand points and background points.
%   Uses the accompanying MEX bundle adjustment code
%
% Inputs:
%   ptMat - an [n,C*4] array of [u,v] coordinates from two cameras and two
%     points [c1u1,c1v1,c2u1,c2v1,c1u2,c1v2,c2u2,c2v2] representing the two
%     ends of a wand of known length
%   UoVo_est - a [1,C*2] array of the (estimated) principal point
%     coordinates for the two cameras
%   wandLen - the known wand length (for scale)
%   ptMat2 (optional) - an [n2,C*2] array of [u,v] coordinates shared
%     between the cameras but not part of the wand
%   f_est - a [1,C] array with estimates of the focal length of each camera
%     (in pixels)
%   nlin (optional) - [1,C*5] array of nonlinear lens distortion
%     coefficients
%   optimMode - determines the number of parameters to optimize:
%     1: camera extrinsics only
%     2: camera extrinsics and focal lengths
%     3: camera extrinsics, focal lengths, and principal points
%   distortionMode - number of non-fixed distortion coefficients

% count number of cameras
nCams=size(ptMat,2)/4;
nPts=size(ptMat,1);
% initialize empty output functions
c=1e24;
xyzMat=ptMat(:,1:6)*NaN;
xyzMat2=zeros(size(ptMat2,1),3)*NaN;
R=repmat(ones(3,3)*NaN,[1 1 nCams-1]);
tv=repmat((1:3)*NaN,[1 1 nCams-1]);
sf=NaN;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Begin preliminary calibration %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Apply undistortion

for i = 1:nCams
  kc=nlin(i*5-4:i*5);
  K = [f(i) 0 UoVo(2*i-1); 0 f(i) UoVo(2*i); 0 0 1]; % Get calibration matrix
  U = [ptMat(:,2*i-1:2*i); ptMat(:,2*nCams+2*i-1:2*nCams+2*i); ptMat2(:,2*i-1:2*i)];
  X = K\[U'; ones(1,length(U(:,1)))]; % get homogeneous points
  x = [X(1,:)./X(3,:); X(2,:)./X(3,:)]; % de-homogenize
  r2 = (x(1,:)).^2 + (x(2,:)).^2; % compute radii to 2nd, 4th, 6th power
  r4 = (r2).^2;
  r6 = (r2).^3;
  % Compute radial distortion
  xd = x.*(ones(2,1)*(1 + kc(1)*r2 + kc(2)*r4 + kc(5)*r6));
  % Tangential distortion components
  a1 = 2.*x(1,:).*x(2,:);
  a2 = r2 + 2*x(1,:).^2;
  a3 = r2 + 2*x(2,:).^2;
  % Compute tangential distortion vector
  xtd = [kc(3)*a1 + kc(4)*a2 ; kc(3) * a3 + kc(4)*a1];
  xd = xd + xtd; % Combine radial and tangential distortion
  % Return to de-normalized coordinates
  uv = [f(i)*xd(1,:) + UoVo(2*i-1); f(i)*xd(2,:) + UoVo(2*i)]';
  ptMatu(:,2*i-1:2*i) = uv(1:nPts,:);
  ptMatu(:,2*nCams+2*i-1:2*nCams+2*i) = uv(nPts+1:2*nPts,:);
  ptMat2u(:,2*i-1:2*i) = uv(2*nPts+1:end,:);
end

%  subtract the principal point
ptMatrm=ptMatu-repmat([UoVo,UoVo],size(ptMatu,1),1);
ptMat2rm=ptMat2u-repmat(UoVo,size(ptMat2u,1),1);
% divide by the focal lengths
for i=1:numel(UoVo)/2
  fArray(1,i*2-1:i*2)=f(i);
end
ptNorm=ptMatrm./repmat(fArray,size(ptMatrm,1),2);
ptNorm2=ptMat2rm./repmat(fArray,size(ptMat2rm,1),1);

nndx1=logical(isfinite(ptNorm(:,1)));
nndx2=logical(isfinite(ptNorm2(:,1)));

% get a Rotation matrix and Translation vector for each camera with respect
% to the last camera - 8-point algorithm
for i=1:nCams-1
  [R(:,:,i),tv(:,:,i)] = twoCamCal_v2([ptNorm(nndx1,i*2-1:i*2), ...
    ptNorm(nndx1,nCams*2-1:nCams*2);ptNorm2(nndx2,[i*2-1,i*2,nCams*2-1,nCams*2])]);
end

if isnan(sum(sum(tv)))
  mess = 'Failed to find an initial estimate of camera params, exiting...';
  disp(mess);
  msgbox(mess);
  return
end
X1 = zeros(size(ptNorm,1),3,nCams-1)*NaN;
X2 = zeros(size(ptNorm,1),3,nCams-1)*NaN;
% Triangulate the estimated 3D position of all points based on the above
% esitmate of camera extrinsics
for i=1:nCams-1
  idx=[i,nCams];
  [X1(:,:,i)] = triangulate_v3(R(:,:,i),tv(:,:,i),ptNorm(:,[idx(1)*2-1:idx(1)*2,idx(2)*2-1:idx(2)*2]));
  [X2(:,:,i)] = triangulate_v3(R(:,:,i),tv(:,:,i),ptNorm(:,[idx(1)*2-1:idx(1)*2,idx(2)*2-1:idx(2)*2]+nCams*2));
end

dWeights=[1,1,1]; % weights array to adjust for accuracy in reconstruction
d = zeros(nCams-1,1)*NaN;
for i=1:nCams-1
  d(i,1)=nanmean(rnorm((X1(:,:,i)-X2(:,:,i)).*repmat(dWeights,size(X1,1),1)));
end

sf=wandLen./d;  % Scale to the expected wand length
for i=1:nCams-1
  X1(:,:,i)=X1(:,:,i).*sf(i);
  X2(:,:,i)=X2(:,:,i).*sf(i);
  tv(:,:,i)=tv(:,:,i).*sf(i);
end
X1=nanmean(X1,3);
X2=nanmean(X2,3);

% no need to scale since tv is scaled above
xyzMat2=triangulate_v3(R,tv,ptNorm2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End preliminary calibration %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set up inputs to run external bundle adjustment starting with the camera
% pose and 3D point locations estimated in the above routines.

sbaXY_tmp=[ptMatu(:,1:nCams*2);ptMatu(:,nCams*2+1:end);ptMat2u];
sbaXYZ_tmp=[X1;X2;xyzMat2];
ndx=find(isfinite(sbaXYZ_tmp(:,1)));
worldXYZ = sbaXYZ_tmp(ndx,:);
imageUV = sbaXY_tmp(ndx,:);
intrinsics=[f', reshape(UoVo,2,nCams)', ones(nCams,1),zeros(nCams,1)];
distortion=reshape(nlin,5,nCams)';
camPos=zeros(nCams,3);
camR=zeros(nCams,4);
for i=1:nCams-1
  camR(i,:)=RMtoQ(R(:,:,i));
  camPos(i,:)=tv(:,:,i);
end
camR(nCams,:)=[1,0,0,0];
distortionFlag=5-distortionMode;
modeList = [4,2,5]; % Number of fixed params for each mode
intrinsicsFlag = modeList(optimMode);

[worldXYZout, camPosout, ~, intrinsicsout, distortionout] = ...
  easySBA(imageUV, worldXYZ, camPos, camR, intrinsics, intrinsicsFlag, ...
  distortion, distortionFlag);

sbaXYZ_tmp(ndx,:) = worldXYZout;
X1sba=sbaXYZ_tmp(1:size(X1,1),:);
X2sba=sbaXYZ_tmp(size(X1,1)+1:size(X1,1)*2,:);
X3sba=sbaXYZ_tmp(size(X1,1)*2+1:end,:);

weightedD=(X1sba-X2sba).*repmat(dWeights,size(X1,1),1);
D=rnorm(weightedD);
sf=wandLen./nanmean(D);

sbaCamPos = camPosout;
for i=1:nCams-1
  tv(:,:,i)=sbaCamPos(i,:)-sbaCamPos(end,:);
end
nlin = reshape(distortionout',1,5*nCams);
f = intrinsicsout(:,1);
UoVo = intrinsicsout(:,2:3);

% optimization function calculation
c=100*nanstd(D)./nanmean(D);
xyzMat=[X1sba,X2sba]*sf;
xyzMat2=X3sba*sf;

*/

/*
function [] = computeCalibrationButton_callback(varargin)
% Main calibration routine - performs complete calibration with given data
% and updates the main gui
% get userdata
h1=findobj('tag','easyWandMain');
uda=get(h1,'userdata');

% Clear the figure window
cla(uda.handles.axes1);

% Make sure we have all the necessary information for a calibration
mess='More information required for a calibration!  Check table inputs';
if ~isfield(uda,'wandPts') || ~isfield(uda,'backgroundPts') || ...
    ~isfield(uda,'wandLen') || ~isfield(uda,'estFocalLengths') || ...
    ~isfield(uda,'imageWidth') || ~isfield(uda,'imageHeight')
  disp(mess);
  msgbox(mess);
  return;
end

% Check validity of data table contents
dTab=get(uda.handles.camTable,'data');
dTabMat=cell2mat(dTab(:,3:5));
for i=1:size(dTab,1)
  dTabMat(i,4)=double(dTab{i,2});
end
dTabMat(dTabMat(:,end)==0,:)=[];
if sum(sum(isnan(dTabMat)))>0
  disp(mess);
  msgbox(mess);
  return;
end

% start gathering the inputs for calibration
ptMat=uda.wandPts;
ptMat2=uda.backgroundPts;
wandLen=uda.wandLen;
f_est=uda.estFocalLengths;
nCams=size(ptMat,2)/4;
if isfield(uda,'pp_est')
  pp_est=uda.pp_est;
else
  pp_est(1:2:nCams*2)=uda.imageWidth./2;
  pp_est(2:2:nCams*2)=uda.imageHeight./2;
end
distortionMode = uda.distortionMode;
optimMode = uda.optimMode;

% check that background points are the appropriate size
if numel(ptMat2)==0
  ptMat2=ones(0,nCams*2);
end

% chop out secondary cameras (& check for a sufficient number of primary
% cameras)
pCams=find(uda.calibrationMode==1);
nPCams = numel(pCams);
if numel(pCams)<2
  mess=('You must have at least two primary cameras. No calibration performed.');
  disp(mess)
  warndlg(mess)
  return
else
  fprintf('Wand calibration will proceed using the primary cameras: ( #s ')
  fprintf('%.0f ',pCams)
  fprintf(')\n')
end
pdx=sort([pCams*2-1,pCams*2]);
p_ptMat=ptMat(:,[pdx,pdx+nCams*2]);
p_pp_est=pp_est(:,pdx);
p_f_est=f_est(:,pCams);
p_ptMat2=ptMat2(:,pdx);
uda.nPCams=nPCams;

% detect and remove rows with any NaNs if allShared == true, only rows with
% 1 or fewer cameras defined if allShared == false
allShared=uda.allShared;
if allShared
  notNaN=logical(sum(isnan(p_ptMat),2)==0);
  notNaN2=logical(sum(isnan(p_ptMat2),2)==0);
else
  notNaN=logical(sum(isfinite(p_ptMat),2)>=5);
  notNaN2=logical(sum(isfinite(p_ptMat2),2)>=3);
end
% Check for points marked for removal, treat as NaN's
rmpts=uda.rmpts; % wand points
bkrmpts=uda.bkrmpts; % background points
p_ptMatrm=p_ptMat;
p_ptMatrm(rmpts(:,1),1:uda.nCams*2)=NaN;
p_ptMatrm(rmpts(:,2),uda.nCams*2+1:end)=NaN;
p_ptMatrm=p_ptMatrm(notNaN,:);
%p_ptMatrm=p_ptMat((notNaN & ~rmpts),:);
p_ptMat2rm=p_ptMat2((notNaN2 & ~bkrmpts),:);
if ~isfield(uda,'nlin_est')
  nlin = zeros(1,5*nPCams);
  uda.nlin_est = nlin;
else
  nlin = uda.nlin_est;
end
Pnlin = zeros(1,5*nPCams);
for i=1:nPCams
  Pnlin(i*5-4:i*5) = nlin(pCams(i)*5-4:pCams(i)*5);
end

set(gcf,'pointer','watch')
ocolor=get(uda.handles.loadButton,'backgroundColor');
set(varargin{1},'backgroundColor',[1,0,0]);
set(varargin{1},'string','Computing the calibration...')
pause(0.05);

% run sparse bundle adjustment calibration routine
[c,xyzMat,R,tv,sf,xyzMat2,ptMatu,ptMat2u,f,UoVo,Pnlin]=sbaCalib(p_ptMatrm,...
  p_pp_est,wandLen,p_ptMat2rm,p_f_est,Pnlin,optimMode,distortionMode);
nlin = zeros(1,nCams*5)*NaN;
for i=1:nPCams
  nlin(pCams(i)*5-4:pCams(i)*5) = Pnlin(i*5-4:i*5);
end

% Make sure the calibration worked
if isnan(sum(sum(tv)))
  msgbox('Calibration Error: failed to find a set of camera positions for the given wand points. Exiting...','Error','error');
  set(gcf,'pointer','arrow')
  set(varargin{1},'string','Compute calibration')
  set(varargin{1},'backgroundColor',ocolor);
  return
end

% reshape UoVo back into easyWand format
UoVo=ceil(reshape(UoVo',1,numel(UoVo)));

% restore original point array size, including NaN rows, replacing removed
% points with NaNs
xyz=zeros(size(ptMat,1),6)*NaN;
%xyz(notNaN & ~uda.rmpts,:)=xyzMat;
%xyz(notNaN & sum(uda.rmpts,2)<2,:)=xyzMat;
xyz(notNaN,:)=xyzMat;
xyz2=zeros(size(ptMat2,1),3)*NaN;
if ~isempty(notNaN2)
  xyz2(notNaN2 & ~uda.bkrmpts,:)=xyzMat2;
else
  xyz2=xyzMat2;
end

set(gcf,'pointer','arrow')
set(varargin{1},'string','Compute calibration')
set(varargin{1},'backgroundColor',ocolor)
disp(' ')
disp('Got an estimate of the focal lengths, camera positions & orientations')
disp(sprintf('Quality: %.2f - smaller is better: [std(wandLen)/mean(wandLen)]*100',c)) %#ok<*DSPS>
disp(' Focal lengths are:')
disp(sprintf('  %d',round(f)))
disp(' Nonlinear distortion coefficients are:')
disp(sprintf('  %.5f',nlin))
disp('Principal Point Coordinates are:')
disp(sprintf('  %d', UoVo));
disp(' ')
disp(' Camera positions are:')
disp(sprintf('  %.5f',reshape(tv,nPCams-1,3)));

% start the conversion to DLT

% gather the calibration points
% Get undistorted points from principal cams and normal points from
% secondary cams
tmp=ptMat;
tmp(notNaN,[pdx,pdx+2*nCams])=ptMatu;
tmp(rmpts(:,1),1:uda.nCams*2)=NaN;
tmp(rmpts(:,2),uda.nCams*2+1:end)=NaN;

if ~isempty(ptMat2)
  tmp2=ptMat2;
  tmp2(notNaN2 & ~bkrmpts,pdx)=ptMat2u;
else
  tmp2=ones(0,nCams*2);
end
ptMat = tmp;
ptMat2=tmp2;

calPts=[ptMat(:,1:size(ptMat,2)/2); ...
  ptMat(:,size(ptMat,2)/2+1:end);ptMat2];

% assemble a calibration frame
frame1=[xyz(:,1:size(xyz,2)/2); ...
  xyz(:,size(xyz,2)/2+1:end);xyz2];

% normalize to the pre-recorded axes
if numel(uda.axisPts)==0
  disp('You do not have any axis points defined.')
  disp('The resulting calibration axes will be euclidean, scaled to the')
  disp('wand length and centered on the cloud of points collected from')
  disp('the wand.  However, they will not be aligned to any particular')
  disp('set of global axes.')
  frame5=frame1-repmat(nanmean(frame1),size(frame1,1),1);
  signFlip=NaN;
  axisXYZ5=[];
  nonOrth=[];
  nAxPts=[];
else % process the orientation axis

  % try and fix dropped rows
  if size(uda.axisPts,2)<nCams*2
    uda.axisPts(:,end+1:nCams*2)=NaN;
  end

  % get number of axis points
  nAxPts=numel(uda.axisPts)/(2*nCams);

  % reshape the axis points in to a column
  if size(uda.axisPts,1)==1
    uda.axisPts=reshape(uda.axisPts,numel(uda.axisPts)/nAxPts,nAxPts)';
  end

  % pull axis points from the back end of the background points array
  axisXYZ=xyz2(end-nAxPts+1:end,:);

  if nAxPts==2
    mess=(['Identified a plumb-line axis, point 1 will be placed at ', ...
      'the origin, point 2 will be on the +Z axis.']);
    hmsg=msgbox(mess);
    uiwait(hmsg);
    frame2=frame1-repmat(axisXYZ(1,:),size(frame1,1),1);
    axisXYZ2=axisXYZ-repmat(axisXYZ(1,:),2,1);

    % rotate such that the axis Z point is on the global Z
    zvec=axisXYZ2(2,:)./rnorm(axisXYZ2(2,:));
    raxis1=cross(zvec,[0,0,1]);
    rang1=acos(dot(zvec,[0,0,1]));
    axisXYZ5=angleaxisRotation(axisXYZ2,repmat(raxis1,2,1),-rang1);
    frame5=angleaxisRotation(frame2,repmat(raxis1,size(frame2,1),1),-rang1);
    signFlip=NaN; % can't evaluate right versus left handed CS
    nonOrth=NaN; % can't evaluate x-y-z orthogonality

  elseif nAxPts==4
    mess=(['Identified a 4 point axis, points 1-4 will correspond to ', ...
      'the origin, +X, +Y & +Z axes.']);
    hmsg=msgbox(mess);
    uiwait(hmsg);

    % compute a missing 1st point (origin) from the other three if necessary
    % or desired
    if isnan(axisXYZ(1,1))
      axisXYZ=findOrigin(axisXYZ);
      disp('Derived an axis origin from the [X,Y,Z] points - examine your data with care')
    else
      yesno=questdlg('Would you like to optimize the axis origin?', ...
        'Optimize origin?','Yes','No','No');
      if strcmp(yesno,'Yes')
        axisXYZ=findOrigin(axisXYZ);
        disp('Derived an axis origin from the [X,Y,Z] points - examine your data with care')
      end
    end

    % center on the axis [0,0,0]
    frame2=frame1-repmat(axisXYZ(1,:),size(frame1,1),1);
    axisXYZ2=axisXYZ-repmat(axisXYZ(1,:),4,1);

    % gather axis points non-orthogonality
    axNorm=axisXYZ2./repmat(rnorm(axisXYZ2),1,3);
    nonOrth(1)=((pi/2)-acos(dot(axNorm(2,:),axNorm(3,:))));
    nonOrth(2)=((pi/2)-acos(dot(axNorm(2,:),axNorm(4,:))));
    nonOrth(3)=((pi/2)-acos(dot(axNorm(4,:),axNorm(3,:))));
    disp(sprintf('Maximum non-orthogonality in the reconstructed axis points was %.2f degrees', ...
      rad2deg(max(abs(nonOrth)))));

    % rotate such that the axis Z point is on the global Z
    zvec=axisXYZ2(4,:)./rnorm(axisXYZ2(4,:));
    raxis1=cross(zvec,[0,0,1]);
    rang1=acos(dot(zvec,[0,0,1]));
    axisXYZ3=angleaxisRotation(axisXYZ2,repmat(raxis1,4,1),-rang1);
    frame3=angleaxisRotation(frame2,repmat(raxis1,size(frame2,1),1),-rang1);

    % rotate so that the X-Y projection of the X axis point is on the global X
    xvec=[axisXYZ3(2,1:2),0];
    xvec=xvec./rnorm(xvec);
    raxis2=cross(xvec,[1,0,0]);
    rang2=acos(dot(xvec,[1,0,0]));
    axisXYZ4=angleaxisRotation(axisXYZ3,repmat(raxis2,4,1),-rang2);
    frame4=angleaxisRotation(frame3,repmat(raxis2,size(frame2,1),1),-rang2);

    % multiply by the sign of the Y axis to change a left-handed coordinate
    % system to right handed (or leave a right handed system alone)
    axisXYZ5=axisXYZ4;
    axisXYZ5(:,2)=axisXYZ4(:,2)*sign(axisXYZ4(3,2));
    frame5=frame4;
    frame5(:,2)=frame4(:,2)*sign(axisXYZ4(3,2));
    signFlip=sign(axisXYZ4(3,2));


  elseif nAxPts>4
    mess=(['Identified axis points for alignment to gravitational acceleration.']);
    hmsg=msgbox(mess);
    uiwait(hmsg);

    % calculate a highly smoothed 2nd derivative - should be constant!
    cnt=0;
    err=[0.1,0.1,0.1];
    keepSmoothing=true;
    while keepSmoothing & cnt<1000
      g=(splineDerivativeKE2(axisXYZ,err,axisXYZ*0+1,2)*uda.gravityFreq^2);
      v=nanstd(g);
      if max(v)>1e-8
        err=err+(v./max(v))*2;
      else
        keepSmoothing=false;
      end
      cnt=cnt+1;
    end
    if cnt==1000
      warndlg(['Failed to identify smooth gravitational acceleration, ',...
        'calibration left unaligned.']);
    else
      % get the zero point
      zeroXYZ=nanmean(axisXYZ);

      % get the average gravity vector
      gVec=nanmean(g)./rnorm(nanmean(g));

      % get the rotation axis to bring gravity onto [0,0,1]
      raxis1=cross(gVec,[0,0,uda.gravityDir]);

      % get the angle between gravity and [0,0,1]
      rang1=acos(dot(gVec,[0,0,uda.gravityDir]));

      % create a new virtual calibration frame that reflects a rotation about
      % raxis1 by the inverse of rang1
      frame5=angleaxisRotation(frame1, ...
        repmat(raxis1,size(frame1,1),1),-rang1);

      zeroXYZ2=angleaxisRotation(zeroXYZ,raxis1,-rang1);

      frame5=frame5-repmat(zeroXYZ2,size(frame1,1),1);

      % legacy
      signFlip=1;
      nonOrth=NaN;
    end

  end
end

% get DLT coefficients and residuals for each camera
coefs = zeros(11,nCams);
rmse = zeros(1,nCams);
for i=1:nCams
  % camera position relative to axes
  [coefs(:,i),rmse(i)]=mdlt_computeCoefficients(frame5,calPts(:,i*2-1:i*2));
end

% reconstruct the two wand points and background points separately explicit
% DLT formulation doesn't support reconstruction of secondary cams
% reconstruct wand points and background points
[xyzR1,rmseR1]=dlt_reconstruct(coefs,ptMat(:,1:size(ptMat,2)/2));
[xyzR2,rmseR2]=dlt_reconstruct(coefs,ptMat(:,size(ptMat,2)/2+1:end));
[xyzR3,rmseR3]=dlt_reconstruct(coefs,ptMat2);
rmseR1(rmpts(:,1),:) = rmseR1(rmpts(:,1),:)*NaN;
rmseR2(rmpts(:,2),:) = rmseR2(rmpts(:,2),:)*NaN;
rmseR3(bkrmpts,:) = rmseR3(bkrmpts,:)*NaN;
disp(' ')
disp('The DLT reprojection errors were:')
disp(sprintf('%.5f ',rmse))

% Compute camera positions based on coordinates
xyzDLT=zeros(3,1,nCams);
T=zeros(4,4,nCams);
for i=1:nCams
  [xyzDLT(:,:,i),T(:,:,i)] = DLTcameraPosition(coefs(:,i));
end

% compute a wand score from the DLT points
dDLT=rnorm(xyzR1-xyzR2);
wandScore=(nanstd(dDLT)./nanmean(dDLT))*100;
wandEndSTD=(nanstd(dDLT))/2^0.5; % estimated standard deviation of each end

fprintf('The DLT wand quality score was: %.2f\n',wandScore);

% Update wand score label
string1=['Wand score: ',num2str(wandScore)];
string2=['Wand end point standard deviation: ',...
  num2str(wandEndSTD),' m'];
string3=['Reprojection errors: ',sprintf('%.2f  ',rmse),' pixels'];
set(uda.handles.wandScoreLabel,'String',sprintf('%s\n',string1,string2,string3));

% Gravity alignment info if relevant
if nAxPts>4

  % grab the gravity values
  gxyz=xyzR3(end-nAxPts+1:end,:);

  % get a smoothed 2nd derivative (should be constant)
  g=(splineDerivativeKE2(gxyz,err,gxyz*0+1,2)*uda.gravityFreq^2);

  % check magnitude
  scaleFactor=nanmean(rnorm(g))./9.80665;


  disp('Final gravity vector:')
  disp(nanmean(g))
  fprintf('\n')
  disp(['You measured gravity as ',num2str(scaleFactor*100),' percent of its expected value.'])

  axisXYZ5=gxyz;
end

% look for bad or outlier points
edx1=find(rmseR1>10*nanmedian(rmseR1));
edx2=find(rmseR2>10*nanmedian(rmseR2));
edx3=find(rmseR3>10*nanmedian(rmseR3));

if numel(edx1>0)
  disp(' ')
  disp('Error analysis suggests that these wand-wave entries may have an')
  disp('incorrectly digitized point #1:')
  disp(sprintf('%.0f ',edx1))
  disp(' ')
end

if numel(edx2>0)
  disp(' ')
  disp('Error analysis suggests that these wand-wave entries may have an')
  disp('incorrectly digitized point #2:')
  disp(sprintf('%.0f ',edx2))
  disp(' ')
end

if numel(edx3>0)
  disp(' ')
  disp('Error analysis suggests that these background entries may have an')
  disp('incorrectly digitized point:')
  disp(sprintf('%.0f ',edx3))
  disp(' ')
end

% Restore principal point size
ppts=pp_est;
ppts(pdx)=UoVo;

% bundle & pass information back
uda.coefs=coefs;
uda.dltRMSE=rmse;
uda.signFlip=signFlip;
uda.preDLT_xyz=xyz;
uda.preDLT_xyz2=xyz2;
uda.frame5=frame5;
uda.axisXYZ5=axisXYZ5;
uda.nonOrth=nonOrth;
uda.focalGAScore=c;
uda.focalLengths=round(f);
uda.rotationMatrices=R;
uda.translationVector=tv;
uda.DLTtranslationVector=xyzDLT;
uda.DLTrotationMatrices=T;
uda.principalPoints=UoVo;
uda.w1 = xyzR1;
uda.w2 = xyzR2;
uda.w3 = xyz(:,1:3);
uda.w4 = xyz(:,4:6);
uda.bkgPts = xyzR3;
uda.bkgPts2 = xyz2;
uda.scaleFactor=sf;
uda.ptMatu=ptMatu;
uda.ptMat2u=ptMat2u;
uda.wandScore=wandScore;
uda.justSaved=0;
uda.nlin=nlin;
uda.ppts = ppts;
set(h1,'userdata',uda);

% Update data in the GUI table
data=get(uda.handles.camTable,'Data');
for i=1:nCams
  data{i,6}=pp_est(2*i-1);
  data{i,7}=pp_est(2*i);
end
for i=1:nPCams
  data{pCams(i),8}=floor(f(i));
  data{pCams(i),9}=UoVo(2*i-1);
  data{pCams(i),10}=UoVo(2*i);
  data{pCams(i),11}=rmse(i);
  data{pCams(i),12}=xyzDLT(1,1,i);
  data{pCams(i),13}=xyzDLT(2,1,i);
  data{pCams(i),14}=xyzDLT(3,1,i);
end
set(uda.handles.camTable,'Data',data);

% plot in the GUI figure axes
plot3DReconstruct();

% Enable save results button and editing button
set(uda.handles.saveResultsButton,'Enable','on');
set(uda.handles.editOutliersButton,'Enable','on');

return

*/
