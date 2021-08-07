package com.swervedrivespecialties.swervelib.motion;

public abstract class MotionProfile {
	private final Goal start;
	private final Goal end;

	public MotionProfile(Goal start, Goal end) {
		this.start = start;
		this.end = end;
	}

	public abstract State calculate(double time);

	public abstract Constraints getConstraints();

	public abstract double getDuration();

	public boolean isFinished(double time) {
		return time > getDuration();
	}

	public Goal getStart() {
		return start;
	}

	public Goal getEnd() {
		return end;
	}

	public static class Constraints {
		public final double maxVelocity;
		public final double maxAcceleration;

		public Constraints(double maxVelocity, double maxAcceleration) {
			this.maxVelocity = maxVelocity;
			this.maxAcceleration = maxAcceleration;
		}
	}

	public static class Goal {
		public final double position;
		public final double velocity;

		public Goal(double position, double velocity) {
			this.position = position;
			this.velocity = velocity;
		}

		public Goal(State state) {
			this(state.position, state.velocity);
		}
	}

	public static class State {
		public final double time;
		public final double position;
		public final double velocity;
		public final double acceleration;

		public State(double time, double position, double velocity, double acceleration) {
			this.time = time;
			this.position = position;
			this.velocity = velocity;
			this.acceleration = acceleration;
		}
	}
}
