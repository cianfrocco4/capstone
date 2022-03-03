/**                                                                                  
 * @file CommonTypes.h                                                               
 * @author Anthony Cianfrocco                                                        
 * @email afc4328@rit.edu                                                            
 */                                                                                  
                                                                                     
#ifndef INCLUDE_COMMONTYPES_H_                                                       
#define INCLUDE_COMMONTYPES_H_

#include <iostream>

namespace CommonTypes
{

typedef struct tsPose
{
    float mrX;
    float mrY;
    float mrTheta;

    tsPose(float arX, float arY, float arTheta) :
        mrX(arX),
        mrY(arY),
        mrTheta(arTheta)
    {}
}tsPose;

/**
 * Structure representing a particle.
 */
typedef struct tsParticle
{
    tsPose msPose;
    float mrProb;

    tsParticle(tsPose asPose, float arProb) :
        msPose(asPose),
        mrProb(arProb)
    {}
}tsParticle;

/**                                                                                  
* Overload of ostream operator of the tsPose struct                                 
*/ 
inline std::ostream& 
operator<<(std::ostream& os, const tsPose& asPose)
{                                                                                    
    os << "(" << asPose.mrX << ", "                                                  
              << asPose.mrY << ", "                                                  
              << asPose.mrTheta  << ")";                                             
                                                                                     
    return os;                                                                       
}   

} /* End Namespace CommonTypes */

#endif /* INCLUDE_COMMONTYPES_H_ */
