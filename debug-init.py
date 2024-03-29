import os
import sys
import numpy as np
import opensim as osim

import sys
# Are we running this script as a test? Users can ignore this line!
# Test update  2
running_as_test = 'unittest' in str().join(sys.argv)

# Define global model where the arm lives.
arm = osim.Model()
if not running_as_test: arm.setUseVisualizer(True)

# ---------------------------------------------------------------------------
# Create two links, each with a mass of 1 kg, centre of mass at the body's
# origin, and moments and products of inertia of zero.
# ---------------------------------------------------------------------------

humerus = osim.Body("humerus",
                    1.0,
                    osim.Vec3(0, 0, 0),
                    osim.Inertia(0, 0, 0))
radius = osim.Body("radius",
                   1.0,
                   osim.Vec3(0, 0, 0),
                   osim.Inertia(0, 0, 0))

# ---------------------------------------------------------------------------
# Connect the bodies with pin joints. Assume each body is 1m long.
# ---------------------------------------------------------------------------

shoulder = osim.PinJoint("shoulder",
                         arm.getGround(), # PhysicalFrame
                         osim.Vec3(0, 0, 0),
                         osim.Vec3(0, 0, 0),
                         humerus, # PhysicalFrame
                         osim.Vec3(0, 1, 0),
                         osim.Vec3(0, 0, 0))

elbow = osim.PinJoint("elbow",
                      humerus, # PhysicalFrame
                      osim.Vec3(0, 0, 0),
                      osim.Vec3(0, 0, 0),
                      radius, # PhysicalFrame
                      osim.Vec3(0, 1, 0),
                      osim.Vec3(0, 0, 0))

# ---------------------------------------------------------------------------
# Add a muscle that flexes the elbow (actuator for robotics people).
# ---------------------------------------------------------------------------

biceps = osim.Millard2012EquilibriumMuscle("biceps",  # Muscle name
                                           200.0,  # Max isometric force
                                           0.6,  # Optimal fibre length
                                           0.55,  # Tendon slack length
                                           0.0)  # Pennation angle
biceps.addNewPathPoint("origin",
                       humerus,
                       osim.Vec3(0, 0.8, 0))

biceps.addNewPathPoint("insertion",
                       radius,
                       osim.Vec3(0, 0.7, 0))

# ---------------------------------------------------------------------------
# Add a controller that specifies the excitation of the muscle.
# ---------------------------------------------------------------------------

brain = osim.PrescribedController()
brain.addActuator(biceps)
brain.prescribeControlForActuator("biceps",
                                  osim.StepFunction(0.5, 3.0, 0.3, 1.0))

# ---------------------------------------------------------------------------
# Build model with components created above.
# ---------------------------------------------------------------------------

arm.addBody(humerus)
arm.addBody(radius)
arm.addJoint(shoulder) # Now required in OpenSim4.0
arm.addJoint(elbow)
arm.addForce(biceps)
arm.addController(brain)

# ---------------------------------------------------------------------------
# Add a console reporter to print the muscle fibre force and elbow angle.
# ---------------------------------------------------------------------------

# We want to write our simulation results to the console.
reporter = osim.ConsoleReporter()
reporter.set_report_time_interval(1.0)
reporter.addToReport(biceps.getOutput("fiber_force"))
elbow_coord = elbow.getCoordinate().getOutput("value")
reporter.addToReport(elbow_coord, "elbow_angle")
arm.addComponent(reporter)

# ---------------------------------------------------------------------------
# Add display geometry. 
# ---------------------------------------------------------------------------

bodyGeometry = osim.Ellipsoid(0.1, 0.5, 0.1)
bodyGeometry.setColor(osim.Gray)
humerusCenter = osim.PhysicalOffsetFrame()
humerusCenter.setName("humerusCenter")
humerusCenter.setParentFrame(humerus)
humerusCenter.setOffsetTransform(osim.Transform(osim.Vec3(0, 0.5, 0)))
humerus.addComponent(humerusCenter)
humerusCenter.attachGeometry(bodyGeometry.clone())

radiusCenter = osim.PhysicalOffsetFrame()
radiusCenter.setName("radiusCenter")
radiusCenter.setParentFrame(radius)
radiusCenter.setOffsetTransform(osim.Transform(osim.Vec3(0, 0.5, 0)))
radius.addComponent(radiusCenter)
radiusCenter.attachGeometry(bodyGeometry.clone())

# ---------------------------------------------------------------------------
# Configure the model.
# ---------------------------------------------------------------------------

state = arm.initSystem()
# Fix the shoulder at its default angle and begin with the elbow flexed.
shoulder.getCoordinate().setLocked(state, True)
elbow.getCoordinate().setValue(state, 0.5 * osim.SimTK_PI)
arm.equilibrateMuscles(state)

# ---------------------------------------------------------------------------
# Simulate.
# ---------------------------------------------------------------------------

manager = osim.Manager(arm)
state.setTime(0)
manager.initialize(state)
state = manager.integrate(10.0)

# ---------------------------------------------------------------------------
# Print/save model file
# ---------------------------------------------------------------------------

arm.printToXML("SimpleArm.osim")
