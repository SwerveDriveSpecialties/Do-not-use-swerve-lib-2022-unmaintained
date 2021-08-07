package com.swervedrivespecialties.swervelib.motion;

public final class TrapezoidalMotionProfile extends MotionProfile {
	private final int direction;

	private final Constraints constraints;
	private final Goal start, end;

	private final double endAccelerationTime, endFullSpeedTime, endDecelerationTime;

	public TrapezoidalMotionProfile(Goal start, Goal end,
	                                Constraints constraints) {
		super(start, end);

		this.direction = shouldFlipAcceleration(start, end) ? -1 : 1;
		start = direct(start);
		end = direct(end);

		this.constraints = constraints;
		this.start = start;
		this.end = end;

		double cutoffBegin = start.velocity / constraints.maxAcceleration;
		double cutoffDistBegin = cutoffBegin * cutoffBegin * constraints.maxAcceleration / 2;

		double cutoffEnd = end.velocity / constraints.maxAcceleration;
		double cutoffDistEnd = cutoffEnd * cutoffEnd * constraints.maxAcceleration / 2;

		double fullTrapezoidDist = cutoffDistBegin + (end.position - start.position) + cutoffDistEnd;
		double accelerationTime = constraints.maxVelocity / constraints.maxAcceleration;

		double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * constraints.maxAcceleration;

		// Handle profiles where the max velocity is never reached
		if (fullSpeedDist < 0) {
			accelerationTime = Math.sqrt(fullTrapezoidDist / constraints.maxAcceleration);
			fullSpeedDist = 0;
		}

		endAccelerationTime = accelerationTime - cutoffBegin;
		endFullSpeedTime = endAccelerationTime + fullSpeedDist / constraints.maxVelocity;
		endDecelerationTime = endFullSpeedTime + accelerationTime - cutoffEnd;
	}

	private static boolean shouldFlipAcceleration(Goal start, Goal end) {
		return start.position > end.position;
	}

	private State direct(State in) {
		return new State(
				in.time,
				in.position * direction,
				in.velocity * direction,
				in.acceleration * direction
		);
	}

	private Goal direct(Goal in) {
		return new Goal(
				in.position * direction,
				in.velocity * direction
		);
	}

	@Override
	public State calculate(double time) {
		double acceleration;
		double velocity;
		double position;

		if (time < endAccelerationTime) {
			acceleration = constraints.maxAcceleration;
			velocity = start.velocity + time * constraints.maxAcceleration;
			position = start.position + (start.velocity + time * constraints.maxAcceleration / 2) * time;
		} else if (time < endFullSpeedTime) {
			acceleration = 0;
			velocity = constraints.maxVelocity;
			position = start.position + (start.velocity + endAccelerationTime * constraints.maxAcceleration / 2) *
					endAccelerationTime + constraints.maxVelocity * (time - endAccelerationTime);
		} else if (time <= endDecelerationTime) {
			acceleration = -constraints.maxAcceleration;
			velocity = end.velocity + (endDecelerationTime - time) * constraints.maxAcceleration;
			double timeLeft = endDecelerationTime - time;
			position = end.position - (end.velocity + timeLeft * constraints.maxAcceleration / 2) * timeLeft;
		} else {
			acceleration = 0;
			velocity = end.velocity;
			position = end.position;
		}

		return direct(new State(time, position, velocity, acceleration));
	}

	@Override
	public Constraints getConstraints() {
		return constraints;
	}

	@Override
	public double getDuration() {
		return endDecelerationTime;
	}
}
