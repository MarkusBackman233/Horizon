#pragma once
#include <vector>
#include "DynamicActor_Internal.h"
#include "StaticActor_Internal.h"
namespace Horizon
{
    class Context {
    public:
        static Context& Instance() {
            static Context instance;
            return instance;
        }
    
        std::vector<DynamicActor_Internal>& GetDynamicActors() { return m_dynamicActors; }
        Actor* CreateDynamicActor(const Transform& pose);
        Actor* CreateStaticActor(const Transform& pose);

    
    private:
        Context();
        std::vector<DynamicActor_Internal> m_dynamicActors;
        std::vector<StaticActor_Internal> m_staticActors;
    };
}
