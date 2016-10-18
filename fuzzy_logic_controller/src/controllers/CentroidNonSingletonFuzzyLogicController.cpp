#include "fl/controllers/CentroidNonSingletonFuzzyLogicController.h"

using namespace std;

namespace fl {

    CentroidNonSingletonFuzzyLogicController::CentroidNonSingletonFuzzyLogicController(const std::string& name) : FuzzyLogicController(name){
        engine = new Engine(name);

        error_proportional = new InputVariable;
        error_proportional->setName("Proportional_Error");
        error_proportional->setRange(-10, 10);
        error_proportional->addTerm(new Gaussian("BIG_NEGATIVE", -10, 3));
        error_proportional->addTerm(new Gaussian("SMALL_NEGATIVE", -5, 3));
        error_proportional->addTerm(new Gaussian("ZERO", 0, 3));
        error_proportional->addTerm(new Gaussian("SMALL_POSITIVE", 5, 3));
        error_proportional->addTerm(new Gaussian("BIG_POSITIVE", 10, 3));
        engine->addInputVariable(error_proportional);

        error_derivative = new InputVariable;
        error_derivative->setName("Derivative_Error");
        error_derivative->setRange(-1, 1);
        error_derivative->addTerm(new Gaussian("BIG_NEGATIVE", -1, 0.3));
        error_derivative->addTerm(new Gaussian("SMALL_NEGATIVE", -0.5, 0.3));
        error_derivative->addTerm(new Gaussian("ZERO", 0, 0.3));
        error_derivative->addTerm(new Gaussian("SMALL_POSITIVE", 0.5, 0.3));
        error_derivative->addTerm(new Gaussian("BIG_POSITIVE", 1, 0.3));
        engine->addInputVariable(error_derivative);

        error_integral = new InputVariable;
        error_integral->setName("Integral_Error");
        error_integral->setRange(-1, 1);
        error_integral->addTerm(new Gaussian("BIG_NEGATIVE", -1, 0.125));
        error_integral->addTerm(new Gaussian("SMALL_NEGATIVE", -0.5, 0.125));
        error_integral->addTerm(new Gaussian("ZERO", 0, 0.125));
        error_integral->addTerm(new Gaussian("SMALL_POSITIVE", 0.5, 0.125));
        error_integral->addTerm(new Gaussian("BIG_POSITIVE", 1, 0.125));
        engine->addInputVariable(error_integral);

        desired_velocity = new OutputVariable;
        desired_velocity->setName("Desired_Velocity");
        desired_velocity->setRange(-1, 1);
        desired_velocity->setDefaultValue(0);
        desired_velocity->addTerm(new Gaussian("VERY_BIG_NEGATIVE", -1, 0.2));
        desired_velocity->addTerm(new Gaussian("BIG_NEGATIVE", -0.75, 0.2));
        desired_velocity->addTerm(new Gaussian("MEDIUM_NEGATIVE", -0.5, 0.2));
        desired_velocity->addTerm(new Gaussian("SMALL_NEGATIVE", -0.25, 0.2));
        desired_velocity->addTerm(new Gaussian("ZERO", 0, 0.2));
        desired_velocity->addTerm(new Gaussian("SMALL_POSITIVE", 0.25, 0.2));
        desired_velocity->addTerm(new Gaussian("MEDIUM_POSITIVE", 0.5, 0.2));
        desired_velocity->addTerm(new Gaussian("BIG_POSITIVE", 0.75, 0.2));
        desired_velocity->addTerm(new Gaussian("VERY_BIG_POSITIVE", 1, 0.2));
        engine->addOutputVariable(desired_velocity);

        RuleBlock* ruleblock = new RuleBlock;
        getRules(ruleblock);
        engine->addRuleBlock(ruleblock);

        engine->configure("Minimum", "", "Minimum", "Maximum", "Centroid");

        string status;
        if (not engine->isReady(&status))
            throw Exception("Engine not ready. The following errors were encountered:\n" + status, FL_AT);
    }
}

