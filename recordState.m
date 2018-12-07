% nomial state
pgiRec = [ pgiRec, x_nom(pgi_nom_index) ];
qgiRec = [ qgiRec, x_nom(qgi_nom_index) ];
vgiRec = [ vgiRec, x_nom(vgi_nom_index) ];
baRec  = [  baRec, x_nom(ba_nom_index) ];
bgRec  = [  bgRec, x_nom(bg_nom_index) ];

pgi1Rec = [ pgi1Rec, x_nom(pgi1_nom_index) ];
qgi1Rec = [ qgi1Rec, x_nom(qgi1_nom_index) ];
pgi2Rec = [ pgi2Rec, x_nom(pgi2_nom_index) ];
qgi2Rec = [ qgi2Rec, x_nom(qgi2_nom_index) ];

% error state
dpgiRec   = [ dpgiRec  ,   dx(pgi_index) ];
dthgiRec  = [ dthgiRec ,  dx(thgi_index) ];
dvgiRec   = [ dvgiRec  ,   dx(vgi_index) ];
dbaRec    = [ dbaRec   ,    dx(ba_index) ];
dbgRec    = [ dbgRec   ,    dx(bg_index) ];

dpgi1Rec  = [ dpgi1Rec ,  dx(pgi1_index) ];
dthgi1Rec = [ dthgi1Rec, dx(thgi1_index) ];
dpgi2Rec  = [ dpgi2Rec ,  dx(pgi2_index) ];
dthgi2Rec = [ dthgi2Rec, dx(thgi2_index) ];

% variance
std_pgiRec  = [ std_pgiRec , sqrt([P(pgi_index(1),pgi_index(1));P(pgi_index(2),pgi_index(2));P(pgi_index(3),pgi_index(3))]) ];
std_thgiRec = [ std_thgiRec, sqrt([P(thgi_index(1),thgi_index(1));P(thgi_index(2),thgi_index(2));P(thgi_index(3),thgi_index(3))]) ];
std_vgiRec  = [ std_vgiRec , sqrt([P(vgi_index(1),vgi_index(1));P(vgi_index(2),vgi_index(2));P(vgi_index(3),vgi_index(3))]) ];
std_baRec   = [ std_baRec  , sqrt([P(ba_index(1),ba_index(1));P(ba_index(2),ba_index(2));P(ba_index(3),ba_index(3))]) ];
std_bgRec   = [ std_bgRec  , sqrt([P(bg_index(1),bg_index(1));P(bg_index(2),bg_index(2));P(bg_index(3),bg_index(3))]) ];

std_pgi1Rec  = [ std_pgi1Rec , sqrt([P(pgi1_index(1),pgi1_index(1));P(pgi1_index(2),pgi1_index(2));P(pgi1_index(3),pgi1_index(3))]) ];
std_thgi1Rec = [ std_thgi1Rec, sqrt([P(thgi1_index(1),thgi1_index(1));P(thgi1_index(2),thgi1_index(2));P(thgi1_index(3),thgi1_index(3))]) ];
std_pgi2Rec  = [ std_pgi2Rec , sqrt([P(pgi2_index(1),pgi2_index(1));P(pgi2_index(2),pgi2_index(2));P(pgi2_index(3),pgi2_index(3))]) ];
std_thgi2Rec = [ std_thgi2Rec, sqrt([P(thgi2_index(1),thgi2_index(1));P(thgi2_index(2),thgi2_index(2));P(thgi2_index(3),thgi2_index(3))]) ];