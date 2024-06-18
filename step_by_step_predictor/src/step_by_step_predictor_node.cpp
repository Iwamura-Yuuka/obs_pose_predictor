#include "step_by_step_predictor/step_by_step_predictor.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "step_by_step_predictor");
  SBSPredictor sbspredictor;
  sbspredictor.process();

  return 0;
}