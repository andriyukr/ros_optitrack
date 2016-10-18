#include "fl/controllers/FuzzyLogicController.h"

using namespace std;

namespace fl {

    FuzzyLogicController::FuzzyLogicController(const std::string& name){
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

        engine->configure("Minimum", "", "Minimum", "Maximum", "Centroid"); // conjunctionT, disjunctionS, activationT, accumulationS, defuzzifierName

        string status;
        if (not engine->isReady(&status))
            throw Exception("Engine not ready. The following errors were encountered:\n" + status, FL_AT);
    }

    FuzzyLogicController::~FuzzyLogicController() {
        delete engine;
    }

    void FuzzyLogicController::getRules(RuleBlock* ruleblock)
     {
         // Zero
         ruleblock->addRule(Rule::parse("if Proportional_Error is NEGATIVE   and Derivative_Error is NEGATIVE    and Integral_Error is ZERO          then Desired_Velocity is SMALL_POSITIVE           ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is NEGATIVE   and Derivative_Error is ZERO        and Integral_Error is ZERO          then Desired_Velocity is SMALL_POSITIVE           ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is NEGATIVE   and Derivative_Error is POSITIVE    and Integral_Error is ZERO          then Desired_Velocity is ZERO               ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is ZERO       and Derivative_Error is NEGATIVE    and Integral_Error is ZERO          then Desired_Velocity is SMALL_POSITIVE           ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is ZERO       and Derivative_Error is ZERO        and Integral_Error is ZERO          then Desired_Velocity is ZERO               ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is ZERO       and Derivative_Error is POSITIVE    and Integral_Error is ZERO          then Desired_Velocity is SMALL_NEGATIVE           ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is POSITIVE   and Derivative_Error is NEGATIVE    and Integral_Error is ZERO          then Desired_Velocity is ZERO               ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is POSITIVE   and Derivative_Error is ZERO        and Integral_Error is ZERO          then Desired_Velocity is SMALL_NEGATIVE           ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is POSITIVE   and Derivative_Error is POSITIVE    and Integral_Error is ZERO          then Desired_Velocity is SMALL_NEGATIVE           ", engine));
         // Negative
         ruleblock->addRule(Rule::parse("if Proportional_Error is NEGATIVE   and Derivative_Error is NEGATIVE    and Integral_Error is NEGATIVE      then Desired_Velocity is BIG_POSITIVE       ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is NEGATIVE   and Derivative_Error is ZERO        and Integral_Error is NEGATIVE      then Desired_Velocity is SMALL_POSITIVE       ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is NEGATIVE   and Derivative_Error is POSITIVE    and Integral_Error is NEGATIVE      then Desired_Velocity is SMALL_NEGATIVE    ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is ZERO       and Derivative_Error is NEGATIVE    and Integral_Error is NEGATIVE      then Desired_Velocity is SMALL_POSITIVE     ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is ZERO       and Derivative_Error is ZERO        and Integral_Error is NEGATIVE      then Desired_Velocity is ZERO               ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is ZERO       and Derivative_Error is POSITIVE    and Integral_Error is NEGATIVE      then Desired_Velocity is SMALL_NEGATIVE               ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is POSITIVE   and Derivative_Error is NEGATIVE    and Integral_Error is NEGATIVE      then Desired_Velocity is SMALL_NEGATIVE       ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is POSITIVE   and Derivative_Error is ZERO        and Integral_Error is NEGATIVE      then Desired_Velocity is SMALL_NEGATIVE     ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is POSITIVE   and Derivative_Error is POSITIVE    and Integral_Error is NEGATIVE      then Desired_Velocity is BIG_NEGATIVE    ", engine));
         // Postive
         ruleblock->addRule(Rule::parse("if Proportional_Error is NEGATIVE   and Derivative_Error is NEGATIVE    and Integral_Error is POSITIVE      then Desired_Velocity is BIG_POSITIVE    ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is NEGATIVE   and Derivative_Error is ZERO        and Integral_Error is POSITIVE      then Desired_Velocity is SMALL_POSITIVE     ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is NEGATIVE   and Derivative_Error is POSITIVE    and Integral_Error is POSITIVE      then Desired_Velocity is SMALL_POSITIVE       ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is ZERO       and Derivative_Error is NEGATIVE    and Integral_Error is POSITIVE      then Desired_Velocity is SMALL_POSITIVE               ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is ZERO       and Derivative_Error is ZERO        and Integral_Error is POSITIVE      then Desired_Velocity is ZERO           ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is ZERO       and Derivative_Error is POSITIVE    and Integral_Error is POSITIVE      then Desired_Velocity is SMALL_NEGATIVE     ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is POSITIVE   and Derivative_Error is NEGATIVE    and Integral_Error is POSITIVE      then Desired_Velocity is SMALL_POSITIVE    ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is POSITIVE   and Derivative_Error is ZERO        and Integral_Error is POSITIVE      then Desired_Velocity is SMALL_NEGATIVE       ", engine));
         ruleblock->addRule(Rule::parse("if Proportional_Error is POSITIVE   and Derivative_Error is POSITIVE    and Integral_Error is POSITIVE      then Desired_Velocity is BIG_NEGATIVE  ", engine));
     }

    scalar FuzzyLogicController::getVelocity(scalar ep, scalar ed, scalar ei){
        error_proportional->setInputValue(max(min(ep, error_proportional->getMaximum()), error_proportional->getMinimum()));
        error_derivative->setInputValue(max(min(ed, error_derivative->getMaximum()), error_derivative->getMinimum()));
        error_integral->setInputValue(max(min(ei, error_integral->getMaximum()), error_integral->getMinimum()));
        engine->process();
        //cout << "[FuzzyLogicController] Controller: " << engine->getName() << endl;
        return desired_velocity->getOutputValue();
    }

    scalar FuzzyLogicController::getVelocity(scalar ep, scalar ed){
        error_proportional->setInputValue(max(min(ep, error_proportional->getMaximum()), error_proportional->getMinimum()));
        error_derivative->setInputValue(max(min(ed, error_derivative->getMaximum()), error_derivative->getMinimum()));
        error_integral->setInputValue(0);
        engine->process();
        //cout << "Input: Proportional Error = " << Op::str(ep) << " and Derivative Error = " << Op::str(ed) << " -> " << "Output: Desired Velocity = " << Op::str(desired_velocity->getOutputValue()) << "\n";
        return desired_velocity->getOutputValue();
    }

    std::string FuzzyLogicController::toString() const {
        return engine->getName();
    }

}

