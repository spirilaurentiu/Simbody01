#ifndef SimTK_SIMBODY_VISUALIZATION_REPORTER_H_
#define SimTK_SIMBODY_VISUALIZATION_REPORTER_H_

/* -------------------------------------------------------------------------- *
 *                      SimTK Core: SimTK Simbody(tm)                         *
 * -------------------------------------------------------------------------- *
 * This is part of the SimTK Core biosimulation toolkit originating from      *
 * Simbios, the NIH National Center for Physics-Based Simulation of           *
 * Biological Structures at Stanford, funded under the NIH Roadmap for        *
 * Medical Research, grant U54 GM072970. See https://simtk.org.               *
 *                                                                            *
 * Portions copyright (c) 2010 Stanford University and the Authors.           *
 * Authors: Peter Eastman                                                     *
 * Contributors:                                                              *
 *                                                                            *
 * Permission is hereby granted, free of charge, to any person obtaining a    *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction, including without limitation  *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,   *
 * and/or sell copies of the Software, and to permit persons to whom the      *
 * Software is furnished to do so, subject to the following conditions:       *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software.                        *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS, CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,    *
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR      *
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE  *
 * USE OR OTHER DEALINGS IN THE SOFTWARE.                                     *
 * -------------------------------------------------------------------------- */


#include "SimTKcommon.h"
#include "simbody/internal/common.h"
#include "simbody/internal/Visualizer.h"

namespace SimTK {

class MultibodySystem;

/**
 * This is an EventReporter that makes it easy to generate on-screen movies of any simulation.
 * Simply create a VisualizationReporter, then invoke addEventReporter() on the MultibodySystem's
 * default subsystem.
 */
class SimTK_SIMBODY_EXPORT VisualizationReporter : public PeriodicEventReporter {
public:
    VisualizationReporter(MultibodySystem& system, Real reportInterval=Infinity);
    VisualizationReporter(MultibodySystem& system, const String& title, Real reportInterval=Infinity);
    ~VisualizationReporter();
    /**
     * Get the Visualizer which generates the images.  It may be used to configure the display.
     */
    const Visualizer& getVisualizer() const;
    Visualizer& updVisualizer();
    void handleEvent(const State& state) const;
    class VisualizationReporterRep;
protected:
    VisualizationReporterRep* rep;
    const VisualizationReporterRep& getRep() const {assert(rep); return *rep;}
    VisualizationReporterRep&       updRep() const {assert(rep); return *rep;}
};

} // namespace SimTK

#endif // SimTK_SIMBODY_VISUALIZATION_REPORTER_H_