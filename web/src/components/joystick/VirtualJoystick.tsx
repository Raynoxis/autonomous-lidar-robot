import { useEffect, useRef, useState } from 'react';
import nipplejs from 'nipplejs';
import type { JoystickManager, JoystickOutputData } from 'nipplejs';
import { useRobotStore } from '../../store';

export const VirtualJoystick = () => {
  const joystickZoneRef = useRef<HTMLDivElement>(null);
  const joystickManagerRef = useRef<JoystickManager | null>(null);
  const [linear, setLinear] = useState(0);
  const [angular, setAngular] = useState(0);

  const { connected, publishVelocity } = useRobotStore();

  useEffect(() => {
    if (!joystickZoneRef.current) return;

    // Create joystick
    const manager = nipplejs.create({
      zone: joystickZoneRef.current,
      mode: 'static',
      position: { left: '50%', top: '50%' },
      color: '#2563eb',
      size: 150,
      restJoystick: true,
      restOpacity: 0.5,
    });

    joystickManagerRef.current = manager;

    // Handle joystick move (vector-based: up=forward, down=reverse, x=steer)
    manager.on('move', (_evt: any, data: JoystickOutputData) => {
      if (!connected) return;

      const maxDistance = 75;
      const distance = Math.min(data.distance, maxDistance) / maxDistance; // Normalize 0-1
      const angle = data.angle.radian;

      // Convert polar to cartesian (right = +x, up = +y in nipple, but we invert y for forward)
      const x = Math.cos(angle) * distance;
      const y = Math.sin(angle) * distance;

      const maxLinear = 0.3; // m/s
      const maxAngular = 1.0; // rad/s

      const linearVel = -y * maxLinear; // push up (y>0) -> forward (+)
      const angularVel = -x * maxAngular; // push right (x>0) -> rotate right (-)

      setLinear(linearVel);
      setAngular(angularVel);

      publishVelocity(linearVel, angularVel);
    });

    // Handle joystick end
    manager.on('end', () => {
      setLinear(0);
      setAngular(0);
      publishVelocity(0, 0);
    });

    return () => {
      if (joystickManagerRef.current) {
        joystickManagerRef.current.destroy();
        joystickManagerRef.current = null;
      }
    };
  }, [connected, publishVelocity]);

  return (
    <div className="flex flex-col items-center">
      <h3 className="font-bold mb-2 text-text-gray text-sm">Virtual Joystick</h3>

      <div
        ref={joystickZoneRef}
        className="w-[200px] h-[200px] bg-dark-card rounded-full border-2 border-dark-border relative"
      />

      <div className="mt-4 w-full bg-dark-card rounded-lg p-3 font-mono text-xs space-y-1">
        <div className="flex justify-between">
          <span className="text-text-gray">Linear:</span>
          <span>{linear.toFixed(2)} m/s</span>
        </div>
        <div className="flex justify-between">
          <span className="text-text-gray">Angular:</span>
          <span>{angular.toFixed(2)} rad/s</span>
        </div>
      </div>
    </div>
  );
};
