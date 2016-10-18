/*
 Author: Juan Rada-Vilela, Ph.D.
 Copyright (C) 2010-2014 FuzzyLite Limited
 All rights reserved

 This file is part of fuzzylite.

 fuzzylite is free software: you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free
 Software Foundation, either version 3 of the License, or (at your option)
 any later version.

 fuzzylite is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License
 for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with fuzzylite.  If not, see <http://www.gnu.org/licenses/>.

 fuzzylite™ is a trademark of FuzzyLite Limited.

 */

#ifndef FL_SINGLETONFUZZYLOGICCONTROLLER_H
#define FL_SINGLETONFUZZYLOGICCONTROLLER_H

#include "fl/controllers/FuzzyLogicController.h"

namespace fl {

    class InputVariable;
    class OutputVariable;
    class Variable;
    class RuleBlock;
    class Hedge;
    class TNorm;
    class SNorm;
    class Defuzzifier;

    class FL_API SingletonFuzzyLogicController : public FuzzyLogicController {
    protected:
        Engine* engine;
        InputVariable* error_proportional;
        InputVariable* error_derivative;
        InputVariable* error_integral;
        OutputVariable* desired_velocity;

    public:
        explicit SingletonFuzzyLogicController(const std::string& name);
    };

}
#endif /* FL_SINGLETONFUZZYLOGICCONTROLLER_H */
