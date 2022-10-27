#pragma once

#include "ecosyslab_export.h"
#include "PlantGrowth.hpp"
using namespace UniEngine;
namespace Orchards {
    class Tree : public IPrivateComponent{
    public:
        TreeGrowthModel m_model;

        void OnInspect() override;

        void OnDestroy() override;

        void OnCreate() override;
    };
}