function install_ellipsoids(path)
%
% Install Ellipsoidal Toolbox.
%

  fprintf('Installing Ellipsoidal Toolbox version 1.1.2 ...\n\n');

  if ~exist('path', 'var')
    path = pwd;
  end
  
  path = '/home/esen/Dropbox/ders/UVA/Research/Toolboxes+Codes/ellipsoids_112_lite/ellipsoids_esen';

  adddir([path]);
  adddir([path '/auxiliary']);
  adddir([path '/control']);
  adddir([path '/control/auxiliary']);
  adddir([path '/demo']);
  adddir([path '/graphics']);
  adddir([path '/solvers']);
  adddir([path '/solvers/gradient']);
  
  fprintf('To finish the installation, go to ''File'' --> ''Set Path...'' and click ''Save''.\n\n');

  return;



  

%%%%%%%%

function adddir(directory)

  if isempty(dir(directory))
    error(['Directory ' directory ' not found!']);
  else
    addpath(directory);
  end

  return;
