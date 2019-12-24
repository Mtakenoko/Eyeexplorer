#include <kdl/chain.hpp>
#include "models.hpp"

namespace KDL{
    Chain EyeExplorer3(){
        Chain eyeexplorer3;
        eyeexplorer3.addChain(Segment());
        return eyeexplorer3;
    }
    
}
