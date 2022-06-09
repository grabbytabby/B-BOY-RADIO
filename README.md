# B-BOY-RADIO
monolithic soundbound technology radio robot
The synthesis of human motion is a complex procedure that involves accurate reconstruction of
movement sequences, modeling of musculoskeletal kinematics, dynamics and actuation, and
characterization of reliable performance criteria. Many of these processes have much in common with the
problems found in robotics research. Task-based methods used in robotics may be leveraged to provide
novel musculoskeletal modeling methods and physiologically accurate performance predictions. In this
paper, we present (i) a new method for the real-time reconstruction of human motion trajectories using
direct marker tracking, (ii) a task-driven muscular effort minimization criterion and (iii) new human
performance metrics for dynamic characterization of athletic skills. Dynamic motion reconstruction is
achieved through the control of a simulated human model to follow the captured marker trajectories in
real-time. The operational space control and real-time simulation provide human dynamics at any
configuration of the performance. A new criteria of muscular effort minimization has been introduced to
analyze human static postures. Extensive motion capture experiments were conducted to validate the new
minimization criterion. Finally, new human performance metrics were introduced to study in details an
athletic skill. These metrics include the effort expenditure and the feasible set of operational space
accelerations during the performance of the skill. The dynamic characterization takes into account skeletal

kinematics as well as muscle routing kinematics and force generating capacities. The developments draw
upon an advanced musculoskeletal modeling platform and a task-oriented framework for the effective
integration of biomechanics and robotics methods.
Index Terms
task-space framework, human motion analysis, robotics, musculoskeletal dynamics, human animation,
operational space formulation
1. INTRODUCTION
In the field of robotics, the motivation to emulate human movement is driven by the
proliferation of humanoid robots and the desire to endow them with human-like movement
characteristics (Nakamura et al.,2003). Inspired by human behaviors, our early work in robot
control encoded tasks and diverse constraints into artificial potential fields capturing human-like
goal-driven behaviors (Khatib and Le Maitre,1978). This concept was later formalized in the task
oriented operational space dynamic framework (Khatib,1986; Khatib,1987). More recently, this
formulation was extended to address whole-body control of humanoid robots and successfully
validated on physical robots (Khatib et al.,2004). The framework provides multi-task prioritized
control architecture allowing the simultaneous execution of multiple objectives in a hierarchical
manner, analogous to natural human motion (see Fig. 1).
One of the major difficulties associated with the prediction and synthesis of human movement
is redundancy resolution. Whether the goal is to gain an understanding of human motion or to
enable synthesis of natural motion in humanoid robots a particularly relevant class of movements
involves targeted reaching. Given a specific target the prediction of kinematically redundant limb
motion is a problem of choosing one of a multitude of control solutions all of which yield
kinematically feasible solutions. It has been observed that humans resolve this redundancy
problem in a relatively consistent manner (Kang et al.,2005; Lacquaniti and Soechting,1982). For
this reason, mathematical models have proven to be valuable tools for motor control prediction
(Hermens and Gielen,2004; Vetter et al.,2002). These models frequently characterize some


Fig. 1. Task-space framework allows the articulated skeleton to maintain balance while accomplishing a manual task
(Khatib et al.,2004).
element of musculoskeletal effort.
Robotics-based effort models frequently utilize quantities that are derivable purely from
skeletal kinematics and that are not specific to muscle actuation. It is thus useful to consider an
analogous measure that encodes information about the overall musculoskeletal system to account
for muscle actuation and its redundancy. Activation, which represents the normalized exertion of
muscles, provides a natural starting point for constructing such a measure. Specifically, the
magnitude of muscle activation vector has been used as an optimization criterion in both static
and dynamic optimizations (Thelen et al.,2003). The utilization of a model-based characterization
of muscle systems, which accounts for muscle kinematic and strength properties, is critical to
authentically simulating human motion since human motions are frequently linked by
physiological constraints.
In this paper, a robotic approach for the synthesis of human motion using a task-space
framework is presented. For this purpose, the direct marker control for human motion
reconstruction, a criterion of task-driven effort minimization and new metrics for dynamic
characterization of human performance were introduced. The result is a dynamic biomechanical
profile of human performance that facilitates the modeling of human motion. These approaches
were tested through extensive motion capture experiments on human subjects including a martial


art master and a professional football player. The results showed that these skillful practitioners
tend to minimize the muscular effort while following the lines of maximum feasible accelerations
when performing a task. These results support the prediction that task-driven human motions
emerge from the use of biomechanical advantage of the human musculoskeletal system under
physiological constraints.
1.1 Task Dynamic Behavior and Control
For a given desired whole-body task of a human-like robot, the motion behaviors should be
specified to be controlled during the execution of the motion. Hand location, balance, effort
minimization, and obstacle and joint limit avoidance are common choices, but the exhaustive list
depends upon the motion to be performed. Considering each behavior as an independent task, the
number of degrees of freedom describing each task is typically less than the number of joints in
the robot. For these situations, there are multiple ways of performing the task. This redundancy is
labeled in solutions as the posture space of the task, containing all possible motions that do not
affect task performance (Khatib et al.,2004). As such, other tasks may be controlled by
selectively choosing the path within the posture space.
In this section, the dynamic model of the task/posture decomposition and the model
describing the motion of the subtask within the posture space (Khatib et al.,2004) are reviewed.
Combination of these two models provides a control structure that compensates for the dynamics
in both spaces, significantly improving performance and responsiveness for multiple tasks.
A task can be defined to be any formal description of desired activity that can be explicitly
represented as a function of the joint coordinates, q, q˙ and q¨. Multiple tasks, xi’s, can be
combined into a single task definition in a higher dimensional space, as long as they are
kinematically consistent with each other. The task coordinates are denoted by xt = xt(q).
The joint space equations of motion can be expressed as,
A(q)¨q + b(q, q˙) + g(q) = Γ, (1)
where q is the n × 1 vector of generalized coordinates, A(q) is the n × n mass matrix, b(q, q˙) is


Choi and Ko,1999] Choi, K., and Ko, H.. (1999). On-line Motion Retargetting. Seventh Pacific
Conference on Computer Graphics and Applications, pp. 32.
[De Sapio et al.,2005] De Sapio, V., Warren, J., Khatib, O., and Delp, S. (2005). Simulating the
Task-level Control of Human Motion: A Methodology and Framework for Implementation.
The Visual Computer, vol. 21, no. 5, pp. 289-302.
[De Sapio et al.,2005] De Sapio, V., Holzbaur, K. R., and Khatib, O. (2005). The Control of
Kinematically Constrained Shoulder Complexes: Physiological and Humanoid Examples.
Proceedings of the 2005 IEEE International Conference on Robotics and Automation,
vol. 1–10, pp. 2952–2959, Barcelona.
[De Sapio et al.,2006] De Sapio, V., Warren, J., Khatib, O. (2006). Predicting Reaching Postures
Using a Kinematically Constrained Shoulder Model. In J. Lenarcic and B. Roth, eds.
Advances in Robot Kinematics, pp. 209-218, Springer.
[Delp et al.,1990] Delp, S.L., Loan, J.P., Hoy, M.G., Zajac, F.E., Topp E.L., Rosen, J.M. (1990)
An interactive graphics-based model of the lower extremity to study orthopaedic surgical
procedures. IEEE Transactions on Biomedical Engineering, vol. 37, pp. 757-767.
[Delp and Loan,1995] Delp, S., and Loan, P. (1995) A software system to develop and analyse
models of musculoskeletal structures. Computers in Biology and Medicine, vol. 25, pp. 21–34.

Khatib, O.a
, Demircan, E.a
, De Sapio, V.a,b, Sentis, L.a
, Besier, T.c
, Delp, S.d
aArtificial Intelligence Laboratory, Stanford University, Stanford, CA 94305, USA
{khatib, emeld, lsentis}@cs.stanford.edu
bSandia National Laboratories, Livermore, CA 94551, USA
vdesap@sandia.gov
cHuman Performance Laboratory, Stanford, CA 94305, USA
besier@stanford.edu
dNeuromuscular Biomechanics Laboratory, Stanford, CA 94305, USA
delp@stanford.edu
