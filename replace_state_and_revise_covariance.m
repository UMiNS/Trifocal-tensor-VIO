function [ x_nom, dx, P ] = replace_state_and_revise_covariance( x_nom, dx, P )
% replace old state by current state and revise covariance matrix 
Tn = [      eye( 7 ), zeros( 7, 9 ), zeros( 7, 7 ), zeros( 7, 7 );
       zeros( 9, 7 ),      eye( 9 ), zeros( 9, 7 ), zeros( 9, 7 );
       zeros( 7, 7 ), zeros( 7, 9 ), zeros( 7, 7 ),      eye( 7 );
            eye( 7 ), zeros( 7, 9 ), zeros( 7, 7 ), zeros( 7, 7 ); ];
        
Te = [      eye( 6 ), zeros( 6, 9 ), zeros( 6, 6 ), zeros( 6, 6 );
       zeros( 9, 6 ),      eye( 9 ), zeros( 9, 6 ), zeros( 9, 6 );
       zeros( 6, 6 ), zeros( 6, 9 ), zeros( 6, 6 ),      eye( 6 );
            eye( 6 ), zeros( 6, 9 ), zeros( 6, 6 ), zeros( 6, 6 ); ];        

x_nom = Tn*x_nom;
dx = Te*dx;

P = Te*P*Te';

end

