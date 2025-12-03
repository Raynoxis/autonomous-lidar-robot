// Type definitions for nipplejs
declare module 'nipplejs' {
  export interface JoystickOutputData {
    distance: number;
    angle: {
      radian: number;
      degree: number;
    };
    direction?: string;
    force: number;
    pressure: number;
    position: {
      x: number;
      y: number;
    };
    instance: JoystickManager;
  }

  export interface JoystickManagerOptions {
    zone: HTMLElement;
    mode?: 'static' | 'dynamic' | 'semi';
    position?: { left?: string; right?: string; top?: string; bottom?: string };
    color?: string;
    size?: number;
    threshold?: number;
    fadeTime?: number;
    multitouch?: boolean;
    maxNumberOfNipples?: number;
    dataOnly?: boolean;
    restJoystick?: boolean | { color?: string; size?: number };
    restOpacity?: number;
    lockX?: boolean;
    lockY?: boolean;
  }

  export interface JoystickManager {
    on(event: 'start' | 'end' | 'move' | 'dir' | 'plain' | 'shown' | 'hidden', handler: (evt: any, data: JoystickOutputData) => void): void;
    off(event: string, handler?: Function): void;
    destroy(): void;
    get(id: number): any;
  }

  function create(options: JoystickManagerOptions): JoystickManager;

  export { create };
}
