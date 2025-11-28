from sys import argv

from simulator import RaceTrack, Simulator, plt

if __name__ == "__main__":
    assert(len(argv) >= 3)
    racetrack = RaceTrack(argv[1], argv[2])
    steps_per_frame = int(argv[3]) if len(argv) >= 4 else 1
    simulator = Simulator(racetrack, steps_per_frame)
    simulator.start()
    plt.show()