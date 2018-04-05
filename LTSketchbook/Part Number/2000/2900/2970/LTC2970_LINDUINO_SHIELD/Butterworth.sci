# create a z-domain butterworth filter approximation in scilab

-->[cells,fact,zzeros,zpoles]=eqiir('lp','butt',[2*%pi/40,16*%pi/40],0.5,0.01)
 zpoles  =
 
    0.8977383 + 0.0927332i    0.8977383 - 0.0927332i  
 zzeros  =
 
  - 1.  - 1.  
 fact  =
 
    0.0047642  
 cells  =
 
                      2          
            1 + 2z + z           
    --------------------------   
                              2  
    0.8145336 - 1.7954767z + z   
 
-->h=fact*poly(zzeros,'z')/poly(zpoles,'z')
 h  =
 
                                       2  
    0.0047642 + 0.0095284z + 0.0047642z   
    -----------------------------------   
                                  2       
        0.8145336 - 1.7954767z + z        
 

 
 