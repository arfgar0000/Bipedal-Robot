=========
Overview
=========

[Last edited by: Arfred Garcia (arfred (dot) garcia (at) outlook (dot) com, 30/07/2026)]

The following documentation outlines the engineering decisions made when designing the bipedal robot (and subsequent PenduBot).
 
The project is still undergoing, however hopefully the following provides a platform for discussion. As this is the first iteration, mistakes are bound to happen (!), so **feel free to message me if you spot fundamental errors in the chain of thought used in each aspect of design - arfred (dot) garcia (at) outlook (dot) com!**

The **overview** below are excerpts taken from the thesis written in 2026. If you aren't interested in the history behind dynamical systems, the PenduBot and the bipedal robot, feel free to look at the other sections in the documentation!

Dynamical systems and chaos theory
-----------------------------------

Dynamics is the study of the evolution of systems, describing processes in motion [1]_ and comprising a broad range of mathematical methods used to model such behaviours [2]_. The same formalisms that capture the swings of pendulums extend to a wide range of domains, including mechanical processes, electricity, thermodynamics, and biological processes such as heartbeat analysis [3]_.

The modern view of dynamics rests on the notion that nature obeys unchanging laws that mathematics can describe [4]_. Newton's work gave this conviction its first concrete vindication, showing that the motion of bodies could be captured by differential equations, which admit closed-form solutions in certain cases. Laplace took this further through his work in celestial mechanics via *Traité de Mécanique Céleste* (1799-1825) [5]_ and probability via *Théorie Analytique des Probabilités* (1812) [6]_. These successes gave him enough confidence that, in his 1814 *Essai Philosophique sur les Probabilités* [7]_, he proposed that an intelligence capable of comprehending every force animating nature would have nothing uncertain before it — the future, like the past, would be laid open to view.

Laplace's confidence proved premature, though not in the way he might have expected. The flaw in the deterministic dream was uncovered not by the introduction of randomness, but from within determinism itself. Working on the three-body problem at the end of the nineteenth century, Poincaré found that even fully deterministic systems governed by smooth equations of motion could exhibit behaviour that defied long-term prediction. Nearly a century after Laplace, in *Science and Method* (1908) [8]_, he observed that tiny differences in initial conditions can produce vastly different outcomes, and that a small error early on can grow into an enormous error later — making prediction effectively impossible. Determinism, in other words, does not entail predictability — a distinction that Laplace's framing had quietly elided.

Poincaré's insight remained largely a curiosity of celestial mechanics until Lorenz, working on a simplified model of atmospheric convection [9]_, stumbled onto the same principle in a different setting by accident: having printed values rounded to three decimal places (0.506) while the computer stored them to six (0.506127), re-entering the rounded value to restart a simulation partway through caused the resulting trajectory to diverge sharply from the original [10]_. The Lorenz system — three coupled ordinary differential equations — exhibited bounded, aperiodic trajectories that depended sensitively on initial conditions, and its attractor's geometry became one of the most recognisable images in the subsequent study of chaos.

Defining chaos precisely, even given the wealth of literature that followed, remains a point without universal agreement [11]_. Identifying chaos across different dynamical systems generally comes down to sensitive dependence on initial conditions under deterministic evolution, together with confinement to a bounded region of state space where trajectories never settle into a fixed or periodic point. Devaney's definition [12]_, one widely cited formulation, extends this to include topological transitivity and dense periodic orbits alongside sensitive dependence, while other treatments instead emphasise the existence of a strange attractor or a positive Lyapunov exponent [13]_. In most cases the differences are real but rarely decisive in practice — the disagreement tends to be about which properties should count as primary, rather than about which systems actually qualify as chaotic.

This same perspective extends well beyond celestial mechanics and idealised convection models. A walking machine is itself a dynamical system: its state evolves under deterministic laws, its steady gaits behave as limit cycles, and, as later sections explore, it too can slip into the sensitive dependence on initial conditions that marks chaos.

Passive dynamics, biomechanics and legged robots
--------------------------------------------------

Legged robots can be characterised as a robot which uses the propulsive force through intermittent frictional contact with the ground to move in space, as opposed to continuous frictional contact with the ground as in wheeled robots. This distinction shapes nearly every aspect of the design problem, from gait planning to balance to mechanical compliance. Legged robots are commonly categorised by the number of limbs they possess - bipeds, quadrupeds, hexapods, and so on - with the choice typically motivated either by a target organism, such as humans or insects, or by the demands of a particular task. Within each category, considerable research has gone into the mechanical design of the leg itself, including notable linkage-based approaches such as the Klann linkage [14]_ and the Strandbeest [15]_. Beyond the design of the leg itself, there is a wealth of literature found in the actuation platform [16]_, compliance of the legs with surrounding nature [17]_, sensors onboard a legged robot and the co-design of these elements [18]_ in a coherent manner so the resulting machine can express the necessary behaviours desired.

There have existed attempts to recreate humanoid automatons (self-operating humanoid robots) since the conceptual design of da Vinci in the late 15th century, where, via pulleys and gears, he tried to replicate the motion of human knights. Although no walking mechanism existed, historians argue this was one of the first attempts to replicate human tendencies via a robotic design, even if at a surface level. It was not until the World Exhibition in 1878 that Chebyshev demonstrated his Plantigrade Machine [23]_, which is considered the world's first machine to replicate the mechanics of a leg. Multiple designs came thereafter, including Yagn's Walking Apparatus (1890) [24]_ and Rygg's Mechanical Horse (1892-93) [25]_. These machines exploited simple mechanisms involving gears and links in a question of kinematics, addressing how one could replicate gaits by combining these elements together. Although, in relation to modern-day dynamics, these seem like crude curiosities of late-1800s experimentation, the lessons laid groundwork for the creation of later celebrated academic tools, such as the Cornell walker [20]_, which derived inspiration from patents by Fallis (1888) [26]_ and Bechstein (1912) [27]_.

Between the late 1800s and the mid-1950s, progress in the dynamics of bipedal walking was comparatively slow; however, historians argue there were key moments that enabled the revitalisation of interest in bipedal robots. Firstly, bipedal walkers required a mathematical framework for dynamic walking. At the Third All-Union Congress of Theoretical and Applied Mechanics, Vukobratović and Juričić introduced the concept of the ZMP (Zero Moment Point), a framework enabling identification of the point where the net horizontal moment of the contact reaction forces vanishes [28]_. For bipedal robots, this enabled the dynamics to be shaped such that the robot remained within its support area, also known as ZMP planning [29]_, providing a sufficient condition for the foot not to roll or tip during the gait. Secondly, as digital computers became more accessible, the ability to realise solutions to large optimisation problems underpinning the study of walking gaits, and to encode this into commercially available electronics, was pivotal.

As with other hardware-intensive industries, the first legged robots were developed under military sponsorship. Mosher [30]_ developed the Walking Truck, a quadruped robot, at General Electric, as part of an effort to build a load-carrying platform capable of traversing terrain inaccessible to wheeled or tracked vehicles; despite its mechanical sophistication, the power and computing limitations of the era restricted it to roughly 5 mph while weighing 3000 lb (1361 kg), and the project was eventually scrapped. Independently, at what is widely regarded as the world's first humanoid robotics laboratory, Ichiro Kato and colleagues at Waseda University developed WABOT-1 in 1973 [31]_, and subsequently produced more than ten bipedal variants over the following fifteen years. The lab's broader significance lay in the 1984 WL-10RD variant, which provided the first experimental verification of the ZMP framework Vukobratović and Juričić had introduced sixteen years earlier, in a practical demonstration [32]_. Legged robotics then became of great interest to the world, driven both by popular fascination with humanoid machines and by the newfound mathematical tractability of dynamic balance. 

This is summarised in the :ref:`table here <bipedal_walking_robots>`, which brings together a list of the various bipeds, collating knowledge derived from Mikolajczyk et al. [33]_, Shi et al. [34]_, Kawaharazuka et al. [22]_ and Pratt [35]_. One of these labs, the MIT LegLab, led by Marc Raibert, led to the creation of Boston Dynamics in 1992, which in 2005 launched the celebrated BigDog robot [36]_, a quadruped mimicking the gait of four-legged animals on rough terrain.

In modern control theory, a significant number of methods employ an overarching notion in which the natural dynamics of a trajectory are cancelled out and then shaped to the desired closed-loop dynamics, which then tracks a reference trajectory [37]_. The ASIMO robot is one of the best-known demonstrations of this philosophy: by tracking a ZMP-stable reference trajectory using position-controlled servo motors at every joint, it produced a smooth, characteristically human-like gait that drew worldwide attention [29]_. The approach is powerful but brittle: it requires an accurate dynamic model of a high-dimensional, multi-actuator system, and identification at this scale is non-trivial in practice, despite a substantial literature on system identification for manipulators and legged platforms. A decade before ASIMO's debut, McGeer had articulated a contrasting philosophy [38]_. Inspired by the analogy of an unpowered glider - which converts altitude into forward motion without a powerplant - McGeer proposed that a biped, suitably designed, could likewise convert gravitational potential into walking motion without active control. He demonstrated that two-legged and kneed mechanisms descending a shallow slope settle into stable limit-cycle gaits driven entirely by gravity, with no actuators, sensors, or control loops - a mechanism he aptly named a 'bipedal glider' [39]_. This paradigm was continued by the team led by Ruina's Lab at Cornell and the Delft group of Wisse, who sought to further refine the baseline models and their variations. This culminated in work bringing these groups together, in which Collins, Ruina, Tedrake and Wisse [40]_ demonstrated that minimally actuated passive-based walkers achieve a specific cost of transport comparable to human walking - roughly 0.2, against ASIMO's ~3.2 (a factor of about sixteen). There are limits to passive techniques: a controlled environment, where the passive walker descends a slope at an angle preset by the experimentalist, is necessary, so robustness with respect to rough, undetermined terrain remains a weak point. However, the rich dynamics that can be studied from a minimal model of this kind provide unique insight into the walking gaits of humans.





.. [1] Oliver Knill. *Dynamical Systems*. Math 118r, Spring semester. 2005.
   url: https://people.math.harvard.edu/~knill/teaching/math118/118_dynamicalsystems.pdf.
.. [2] Philip Holmes. *A Short History of Dynamical Systems Theory: 1885-2007*.
   History of Mathematics article. Princeton, NJ, USA, 2007.
.. [3] Steven H. Strogatz. *Nonlinear Dynamics and Chaos: With Applications to
   Physics, Biology, Chemistry, and Engineering*. 2nd ed. Studies in Nonlinearity.
   Boulder, CO: Westview Press, 2015.
.. [4] Boris Hasselblatt and Anatole Katok. *A First Course in Dynamics: With a
   Panorama of Recent Developments*. Cambridge University Press, 2003.
.. [5] Pierre-Simon Laplace. *Traité de Mécanique Céleste*. Five volumes
   published between 1799 and 1825. Paris: Chez J. B. M. Duprat, 1799-1825.
.. [6] Pierre-Simon Laplace. *Théorie Analytique des Probabilités*. Paris:
   Courcier, 1812.
.. [7] Pierre-Simon Laplace. *Essai Philosophique sur les Probabilités*. Paris:
   Courcier, 1814.
.. [8] Henri Poincaré. *The Foundations of Science: Science and Hypothesis, The
   Value of Science, Science and Method*. Ed. by Josiah Royce. Trans. by George
   Bruce Halsted. Cambridge: Cambridge University Press, 2015.
   isbn: 9781107252950. doi: 10.1017/CBO9781107252950.
.. [9] Edward N. Lorenz. "Deterministic Nonperiodic Flow". In: *Journal of the
   Atmospheric Sciences* 20.2 (1963), pp. 130-141.
   doi: 10.1175/1520-0469(1963)020<0130:DNF>2.0.CO;2.
.. [10] Edward N. Lorenz. *The Essence of Chaos*. The Jessie and John Danz
   Lectures. Seattle: University of Washington Press, 1993.
.. [11] Baghdadi Hamidouche, Kamel Guesmi, and Najib Essounbouli. "Mastering
   chaos: A review".
.. [12] Robert L. Devaney. *An Introduction to Chaotic Dynamical Systems*. 2nd
   ed. First published 1989 by Addison-Wesley. Boulder, CO: Westview Press, 2003.
.. [13] Alan Wolf et al. "Determining Lyapunov Exponents from a Time Series".
   In: *Physica D: Nonlinear Phenomena* 16.3 (1985), pp. 285-317.
   doi: 10.1016/0167-2789(85)90011-9.
.. [14] Joseph C. Klann. *Walking Device*. U.S. Patent. Filed: November 11, 1998. July 2001.
   url: https://patents.google.com/patent/US6260862B1.
.. [15] Theo Jansen. *Strandbeest*. Accessed: 2026-06-01. 2007.
   url: https://www.strandbeest.com.
.. [16] Benjamin G. Katz. "A Low Cost Modular Actuator for Dynamic Robots". Master's
   thesis. Department of Mechanical Engineering: Massachusetts Institute of Technology, 2018.
.. [17] Robin Bendfeld and C. David Remy. *Bipedal Walking with Continuously Compliant
   Robotic Legs*. 2024. arXiv: 2411.06948 [cs.RO]. url: https://arxiv.org/abs/2411.06948.
.. [18] Adrian B. Ghansah et al. *Humanoid Robot Co-Design: Coupling Hardware Design
   with Gait Generation via Hybrid Zero Dynamics*. 2023. arXiv: 2308.10962 [cs.RO].
   url: https://arxiv.org/abs/2308.10962.
.. [20] Steven H. Collins, Martijn Wisse, and Andy Ruina. "A Three-Dimensional Passive-
   Dynamic Walking Robot with Two Legs and Knees". In: *International Journal of
   Robotics Research* 20.2 (2001), pp. 607-615.
.. [22] Kento Kawaharazuka et al. "MEVITA: Open-Source Bipedal Robot Assembled from
   E-Commerce Components via Sheet Metal Welding". In: *Proceedings of the 2025
   IEEE-RAS International Conference on Humanoid Robots*. 2025. arXiv: 2508.17684 [cs.RO].
.. [23] Pafnuty L. Chebyshev. *Plantigrade Machine*. Exhibited at the Exposition
   Universelle, Paris. Mechanical walking linkage; original preserved at the
   Polytechnic Museum, Moscow. 1878.
.. [24] Nicholas Yagn. *Apparatus for Facilitating Walking, Running, and Jumping*. U.S.
   Patent 420,178. Filed June 13, 1889; issued January 28, 1890. Jan. 1890.
.. [25] Lewis A. Rygg. *Mechanical Horse*. U.S. Patent 491,927. Filed April 8, 1892;
   issued February 14, 1893. Feb. 1893.
.. [26] George T. Fallis. *Improvement in Walking Toys*. U.S. Patent 376,588. 1888.
.. [27] Bechstein. *Improvements in and Relating to Toys*. U.K. Patent 7453. 1912.
.. [28] Miomir Vukobratović and Davor Juričić. "Contribution to the Synthesis of Biped
   Gait". In: *Proceedings of the Third All-Union Congress of Theoretical and Applied
   Mechanics*. Moscow, USSR, 1968.
.. [29] Russ Tedrake. *Underactuated Robotics: Algorithms for Walking, Running,
   Swimming, Flying, and Manipulation*. Version 1.0. MIT OpenCourseWare, 2022.
   url: http://underactuated.mit.edu/.
.. [30] Ralph S. Mosher. *Handyman to Hardiman*. SAE Technical Paper 670088. Describes
   the man-amplifier programme that led to the Walking Truck. General Electric Company,
   1967. doi: 10.4271/670088.
.. [31] Hun-ok Lim and Atsuo Takanishi. "Biped Walking Robots Created at Waseda
   University: WL and WABIAN Family". In: *Philosophical Transactions of the Royal
   Society A* 365.1850 (2007). Surveys the WL (Waseda Leg) and WABIAN bipedal lineage
   1967-2006, more than ten variants, pp. 49-64. doi: 10.1098/rsta.2006.1920.
.. [32] Atsuo Takanishi et al. "The Realization of Dynamic Walking by the Biped Walking
   Robot WL-10RD". In: *Proceedings of the International Conference on Advanced Robotics
   (ICAR'85)*. Tokyo, Japan, 1985.
.. [33] T. Mikolajczyk et al. "Recent Advances in Bipedal Walking Robots: Review of
   Gait, Drive, Sensors and Control Systems". In: *Sensors* 22.12 (2022), p. 4440.
   doi: 10.3390/s22124440.
.. [34] Haochen Shi et al. "ToddlerBot: Open-Source ML-Compatible Humanoid Platform for
   Loco-Manipulation". In: (2024). Equal contribution: Shi, Wang; Equal advising: Song, Liu.
.. [35] Jerry E. Pratt. "Exploiting Inherent Robustness and Natural Dynamics in the
   Control of Bipedal Walking Robots". Department of Electrical Engineering and Computer
   Science. Ph.D. thesis. Cambridge, MA: Massachusetts Institute of Technology, June 2000.
.. [36] Marc Raibert et al. "BigDog, the Rough-Terrain Quadruped Robot". In: *Proceedings
   of the 17th IFAC World Congress*. 2008, pp. 10822-10825.
   doi: 10.3182/20080706-5-KR-1001.01833.
.. [37] Mark W. Spong, Seth Hutchinson, and M. Vidyasagar. *Robot Modeling and Control*.
   2nd ed. John Wiley & Sons, 2020. isbn: 9781119523994.
.. [38] Tad McGeer. "Passive Dynamic Walking". In: *The International Journal of
   Robotics Research* 9.2 (1990), pp. 62-82. doi: 10.1177/027836499000900206.
.. [39] Tad McGeer. "Powered Flight, Child's Play, Silly Wheels, and Walking Machines".
   In: *Proceedings of the IEEE International Conference on Robotics and Automation
   (ICRA)*. Cincinnati, OH, USA, 1990, pp. 1592-1597. doi: 10.1109/ROBOT.1990.126234.
.. [40] Steve Collins et al. "Efficient Bipedal Robots Based on Passive-Dynamic
   Walkers". In: *Science* 307.5712 (2005), pp. 1082-1085.
   doi: 10.1126/science.1107799.

