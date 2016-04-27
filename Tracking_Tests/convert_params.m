%Script to convert found transformation matrix from camera to robot base to
%its inverse usable form
Rotm=tform2rotm(TBase);
Tvec=tform2trvec(TBase);
invRotm=inv(Rotm)
invTvec=-1*Tvec