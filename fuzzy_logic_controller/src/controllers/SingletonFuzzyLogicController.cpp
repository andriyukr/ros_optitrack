#include "fl/controllers/SingletonFuzzyLogicController.h"

using namespace std;

namespace fl {

    SingletonFuzzyLogicController::SingletonFuzzyLogicController(const std::string& name) : FuzzyLogicController(name){
        engine = new Engine(name);

        error_proportional = new InputVariable;
        error_proportional->setName("Proportional_Error");
        error_proportional->setRange(-1, 1);
        error_proportional->addTerm(new Gaussian("NEGATIVE", -1, 0.5));
        error_proportional->addTerm(new Gaussian("ZERO", 0, 0.5));
        error_proportional->addTerm(new Gaussian("POSITIVE", 1, 0.5));
        engine->addInputVariable(error_proportional);

        error_derivative = new InputVariable;
        error_derivative->setName("Derivative_Error");
        error_derivative->setRange(-1, 1);
        error_derivative->addTerm(new Gaussian("NEGATIVE", -1, 0.5));
        error_derivative->addTerm(new Gaussian("ZERO", 0, 0.5));
        error_derivative->addTerm(new Gaussian("POSITIVE", 1, 0.5));
        engine->addInputVariable(error_derivative);

        error_integral = new InputVariable;
        error_integral->setName("Integral_Error");
        error_integral->setRange(-1, 1);
        error_integral->addTerm(new Gaussian("NEGATIVE", -1, 0.5));
        error_integral->addTerm(new Gaussian("ZERO", 0, 0.5));
        error_integral->addTerm(new Gaussian("POSITIVE", 1, 0.5));
        engine->addInputVariable(error_integral);

        desired_velocity = new OutputVariable;
        desired_velocity->setName("Desired_Velocity");
        desired_velocity->setRange(-2, 2);
        desired_velocity->setDefaultValue(0);
        desired_velocity->addTerm(new Gaussian("BIG_NEGATIVE", -2, 0.5));
        desired_velocity->addTerm(new Gaussian("SMALL_NEGATIVE", -1.5, 0.5));
        desired_velocity->addTerm(new Gaussian("ZERO", 0, 0.5));
        desired_velocity->addTerm(new Gaussian("SMALL_POSITIVE", 1.5, 0.5));
        desired_velocity->addTerm(new Gaussian("BIG_POSITIVE", 2, 0.5));
        engine->addOutputVariable(desired_velocity);

        RuleBlock* ruleblock = new RuleBlock;
        getRules(ruleblock);
        engine->addRuleBlock(ruleblock);

        engine->configure("Minimum", "", "Minimum", "Maximum", "Centroid"); // conjunctionT, disjunctionS, activationT, accumulationS, defuzzifierName, resolution

        string status;
        if (not engine->isReady(&status))
            throw Exception("Engine not ready. The following errors were encountered:\n" + status, FL_AT);
    }
}

