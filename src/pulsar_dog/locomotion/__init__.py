"""Policy-driven locomotion, structured the way a simulation environment is.

The layout mirrors Isaac Lab's manager-based environments so a controller
developed in simulation transfers without rewriting its plumbing:

* :mod:`~pulsar_dog.locomotion.commands` - velocity command sampling
* :mod:`~pulsar_dog.locomotion.observations` - the observation vector, term by term
* :mod:`~pulsar_dog.locomotion.policy` - what maps observations to actions
* :mod:`~pulsar_dog.locomotion.terminations` - when to stop, and how hard
* :mod:`~pulsar_dog.locomotion.runner` - the decimated policy loop

It sits entirely on top of :class:`~pulsar_dog.robot.PulsarDog`: the action space
is the body velocity command, so every limit, ramp and watchdog still applies.
"""

from pulsar_dog.locomotion.commands import (
    FixedVelocityCommand,
    UniformVelocityCommand,
    VelocityCommandCfg,
)
from pulsar_dog.locomotion.observations import (
    ObsContext,
    ObservationManager,
    ObservationTerm,
    projected_gravity,
    tilt_angle,
)
from pulsar_dog.locomotion.policy import (
    CommandFollowingPolicy,
    PatrolPolicy,
    Policy,
    TorchScriptPolicy,
    ZeroPolicy,
)
from pulsar_dog.locomotion.runner import (
    ActionCfg,
    LocomotionCfg,
    LocomotionRunner,
    RunResult,
)
from pulsar_dog.locomotion.terminations import (
    Termination,
    TerminationManager,
    TerminationTerm,
)

__all__ = [
    "ActionCfg",
    "CommandFollowingPolicy",
    "FixedVelocityCommand",
    "LocomotionCfg",
    "LocomotionRunner",
    "ObsContext",
    "ObservationManager",
    "ObservationTerm",
    "PatrolPolicy",
    "Policy",
    "RunResult",
    "Termination",
    "TerminationManager",
    "TerminationTerm",
    "TorchScriptPolicy",
    "UniformVelocityCommand",
    "VelocityCommandCfg",
    "ZeroPolicy",
    "projected_gravity",
    "tilt_angle",
]
