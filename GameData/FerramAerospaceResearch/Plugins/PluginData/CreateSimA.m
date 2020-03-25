% CREATESIMA
%  Linear derivative matrix for oscillation simulation.
%  [longA, latA, T, x_long, x_lat] = CreateSimA (j, T, x0_long, x0_lat)
%  
%  The stability derivatives, arranged in two matrices, that can be used to
%  predict perturbation reaction, in particular to check for susceptibility to
%  oscillations and fast dynamic instability.
%  
%  Parameters
%   j - data point index. If you have say 12 exports from KSP, then j=11 will
%       be the second to last exported static derivatives data set, 
%       i.e. data{11} from sdexport.txt.
%       Default value is the last index of the data.
%   T - time period for 'simulation' (if any). Can be a scalar, eg. T=10 means
%       10 seconds with default spacing (dt=0.01). Can be a row vector with
%       timepoints, e.g. 0:0.005:60. The vector must start at T(1)=0.
%       Default value is 10 seconds.
%   x0_long - initial (upset) condition as column vector [w, u, q, theta]',
%       where w and u is in m/s, q is in deg/s and theta in deg.
%       Default, if x0_long and x0_lat are left blank, is [1;0;0;0].
%   x0_lat - initial (upset) condition as column vector [beta, roll rate,
%       yaw rate, bank angle]; beta and bank in deg and rates in deg/s.
%       Default, if x0_long and x0_lat are left blank, is [1;0;0;0].
%  
%  Results
%   longA - Derivative matrix for longitudinal simulation.
%   latA - Derivative matrix for lateral simulation.
%   T - Actual time points used. T(1) is the time of the initial condition.
%   x_long - Four row vectors, use plot(T,x_long) for longitudinal plot.
%   x_lat - Four row vectors, use plot(T,x_lat) for lateral plot.
%  
function [longA, latA, T, x_long, x_lat] = CreateSimA (j, T, x0_long, x0_lat)
  load sdexport.txt data
  if ~exist("data") || ~iscell(data) || length(data) < 1,
    error("Could not load data from sdexport.txt file.");
    endif
  
  if nargin < 1,
    j = length(data);
    elseif  j < 1 || j > length(data),
    error(["Invalid idx j=" num2str(j) "."]);
    endif
  if nargin < 2,
    T = 10;
    endif
  if isscalar(T),
    T = 0:0.01:T;
    endif
  if nargin < 3,
    x0_long = [1; 0; 0; 0];
    endif
  if nargin < 3,
    x0_lat = [1; 0; 0; 0];
    elseif nargin < 4,
    error("Missing parameter x0_lat.");
    endif
  
  if isrow(T) && iscolumn(x0_long) && iscolumn(x0_lat) ...
      && length(T) > 1 && length(x0_long) == 4 && length(x0_lat) == 4,
    x0_long(3:4) = x0_long(3:4) * pi/180;
    x0_lat = x0_lat * pi/180;
    else
    error("Parameter error in T, x0_long or x0_lat!");
    endif
  
  longA = LongitudinalA(data{j});
  latA = LateralA(data{j});
  
  x_long = NaN(4,length(T));
  x_lat = x_long;
  x_long(:,1) = x0_long;
  x_lat(:,1) = x0_lat;
  dt = diff(T);
  
  for k=1:length(dt),
    x_long(:,k+1) = x_long(:,k) + dt(k)*longA*x_long(:,k);
    x_lat(:,k+1) = x_lat(:,k) + dt(k)*latA*x_lat(:,k);
  endfor
  
  x_long(3:4,:) = x_long(3:4,:) * 180/pi;
  x_lat = x_lat * 180/pi;
  
  if nargin < 1 || nargin > 1,
    clf;
    yellow = "k"; % black because yellow is difficult to see on white.
    subplot(2,1,1); x = x_long;
    plot(T,x(1,:),"g", T,x(2,:),yellow, T,x(3,:),"r", T, x(4,:), "c");
    grid on; legend("w", "u", "q", "theta");
    subplot(2,1,2); x = x_lat;
    plot(T,x(1,:),"g", T,x(2,:),yellow, T,x(3,:),"r", T, x(4,:), "c");
    grid on; legend("beta", "roll rate", "yaw rate", "bank angle");
  endif
endfunction


function [A] = LongitudinalA(d)
  mac2u = d.craft.chord / (2*d.env.speed); % can be used for backward compability
  mac2u = 1; % Notice: The 'mac/(2u)' scaling is NOT used in the current version!
  
  ZXMw = [d.deriv.Zw, d.deriv.Xw, d.deriv.Mw]';
  ZXMu = [d.deriv.Zu, d.deriv.Xu, d.deriv.Mu]';
  ZXMq = mac2u * [d.deriv.Zq, d.deriv.Xq, d.deriv.Mq]';
  
  A = [ ZXMw, ZXMu, ZXMq + [d.env.speed;0;0], [0; -d.env.effg; 0]; ...
        0, 0, 1, 0 ];
endfunction


function [A] = LateralA (d)
  u0 = d.env.speed;
  b2u = d.craft.span / (2*u0); % can be used for backward compability
  b2u = 1; % Notice: The 'b/(2u)' scaling is NOT used in the current version!
  factor_xz_x = d.craft.inertia.lxz / d.craft.inertia.lxx;
  factor_xz_z = d.craft.inertia.lxz / d.craft.inertia.lzz;
  factor_invxz = 1 / (1 - factor_xz_x * factor_xz_z);
  
  % first let us do rate scaling without any other transformation
  YLNb = [d.deriv.Yb, d.deriv.Lb, d.deriv.Nb]';
  YLNp = b2u * [d.deriv.Yp, d.deriv.Lp, d.deriv.Np]';
  YLNr = b2u * [d.deriv.Yr, d.deriv.Lr, d.deriv.Nr]';
  
  % apply the I_x_z factors (often negligible)
  I_x_z = factor_invxz * [1, factor_xz_x; factor_xz_z, 1];
  YLNb(2:3) = I_x_z * YLNb(2:3);
  YLNp(2:3) = I_x_z * YLNp(2:3);
  YLNr(2:3) = I_x_z * YLNr(2:3);
  
  % produce matrix
  g = cos(pi/180*d.deriv.AoA) * d.env.effg;
  A = [ YLNb(1)/u0, YLNp(1)/u0, YLNr(1)/u0 - 1, g/u0; ...
        YLNb(2:3),  YLNp(2:3),  YLNr(2:3),      [0;0]; ...
        0, 1, 0, 0];
endfunction
