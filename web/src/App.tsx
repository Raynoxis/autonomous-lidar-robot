import { useEffect } from 'react';
import { Header } from './components/Header';
import { Button, Panel } from './components/ui';
import { MapViewer } from './components/map';
import { VirtualJoystick } from './components/joystick';
import { SystemStatus } from './components/SystemStatus';
import { useRobotStore } from './store';
import { useButtonStates } from './hooks';
import { YoutubeFooter } from './components/YoutubeFooter';

function App() {
  const {
    connected,
    rosUrl,
    setRosUrl,
    connect,
    disconnect,
    commandLog,
    emergencyStop,
    startExploration,
    stopExploration,
  } = useRobotStore();

  const buttonStates = useButtonStates();

  useEffect(() => {
    // Cleanup on unmount
    return () => {
      if (connected) {
        disconnect();
      }
    };
  }, []);

  return (
    <div className="min-h-screen flex flex-col bg-dark-darker">
      <Header />

      <div className="flex-1 grid grid-cols-1 lg:grid-cols-[350px_1fr_350px] gap-4 p-4 overflow-hidden">
        {/* Left Panel - Connection & Control */}
        <Panel title="Connection & Control" className="overflow-hidden">
          <div className="space-y-4">
            {/* Connection */}
            <div>
              <label className="block text-sm text-text-gray mb-2">ROS Bridge URL</label>
              <input
                type="text"
                value={rosUrl}
                onChange={(e) => setRosUrl(e.target.value)}
                disabled={connected}
                className="w-full px-3 py-2 bg-dark-card border border-dark-border rounded-lg text-text-light focus:outline-none focus:border-primary disabled:opacity-50"
              />
            </div>

            <div className="flex gap-2">
              <Button
                variant="success"
                fullWidth
                disabled={!buttonStates.canConnect}
                onClick={connect}
              >
                Connect
              </Button>
              <Button
                variant="danger"
                fullWidth
                disabled={!buttonStates.canDisconnect}
                onClick={disconnect}
              >
                Disconnect
              </Button>
            </div>

            {/* Emergency Stop */}
            <Button
              variant="danger"
              fullWidth
              disabled={!buttonStates.canEmergencyStop}
              onClick={emergencyStop}
            >
              ⚠ EMERGENCY STOP
            </Button>

            <hr className="border-dark-border" />

            {/* Virtual Joystick */}
            <VirtualJoystick />

            {/* Dynamic System Status */}
            <SystemStatus />
          </div>
        </Panel>

        {/* Center Panel - Map */}
        <Panel title="Interactive Map" className="overflow-hidden">
          <div className="h-full">
            <MapViewer />
          </div>
        </Panel>

        {/* Right Panel - Commands */}
        <Panel title="Navigation Commands" className="overflow-hidden">
          <div className="space-y-4">
            {/* Exploration */}
            <div>
              <h3 className="font-bold mb-2 text-text-gray text-sm uppercase">Exploration</h3>
              <div className="space-y-2">
                <Button
                  variant="primary"
                  fullWidth
                  size="sm"
                  disabled={!buttonStates.canStartExplore}
                  onClick={startExploration}
                >
                  🔍 Start Exploration
                </Button>
                <Button
                  variant="danger"
                  fullWidth
                  size="sm"
                  disabled={!buttonStates.canStopExplore}
                  onClick={stopExploration}
                >
                  ⏹️ Stop Exploration
                </Button>
              </div>
            </div>

            <hr className="border-dark-border" />

            {/* Command Log */}
            <div>
              <h3 className="font-bold mb-2 text-text-gray text-sm uppercase">Command Log</h3>
              <div className="bg-dark-card rounded-lg p-2 max-h-[400px] overflow-y-auto font-mono text-xs space-y-1">
                {commandLog.length === 0 ? (
                  <div className="text-text-gray text-center py-4">No commands yet</div>
                ) : (
                  commandLog.map((entry, index) => (
                    <div key={index} className="text-text-gray border-b border-dark-border pb-1 last:border-b-0">
                      <span className="text-text-light font-bold">[{entry.timestamp}]</span>{' '}
                      {entry.message}
                    </div>
                  ))
                )}
              </div>
            </div>
          </div>
        </Panel>
      </div>

      <YoutubeFooter />
    </div>
  );
}

export default App;
