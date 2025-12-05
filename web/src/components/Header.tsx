import React from 'react';

export const Header: React.FC = () => {
  return (
    <div className="bg-dark-bg px-8 py-4 border-b-2 border-dark-border">
      <div className="flex flex-col items-center text-center gap-2">
        <div className="text-2xl font-bold text-text-light flex items-center gap-2">
          <span role="img" aria-label="robot">🤖</span>
          <span>MakersPet Mini - Control Center</span>
        </div>
        <div className="text-sm text-text-gray">
          Web UI autonome pour ROS2/Nav2 + SLAM
        </div>
      </div>
    </div>
  );
};
