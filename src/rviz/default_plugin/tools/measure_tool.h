/*
 * measure_tool.h
 *
 *  Created on: Aug 8, 2012
 *      Author: gossow
 */

#ifndef MEASURE_TOOL_H_
#define MEASURE_TOOL_H_

#include "rviz/tool.h"

namespace rviz
{

class Shape;

class MeasureTool : public Tool
{
public:
  MeasureTool();
  virtual
  ~MeasureTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent( ViewportMouseEvent& event );
private:

  Shape* sphere_;
};

} /* namespace rviz */
#endif /* MEASURE_TOOL_H_ */
