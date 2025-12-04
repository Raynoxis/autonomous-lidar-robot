import React from 'react';
import { useSystemStatus } from '../hooks/useSystemStatus';
import type { SystemStatusItem } from '../types';
import { Panel } from './ui';

const StatusIcon: React.FC<{ status: SystemStatusItem['status'] }> = ({ status }) => {
  const getStyle = () => {
    switch (status) {
      case 'success':
        return 'bg-green-500';
      case 'checking':
        return 'bg-yellow-500 animate-pulse';
      case 'error':
        return 'bg-red-500';
      case 'pending':
        return 'bg-gray-400';
      case 'disabled':
        return 'bg-gray-600';
    }
  };

  return <div className={`w-3 h-3 rounded-full ${getStyle()}`} />;
};

export const SystemStatus: React.FC = () => {
  const statusItems = useSystemStatus();

  return (
    <Panel title="État Système">
      <div className="space-y-3">
        {statusItems.map((item, index) => (
          <div
            key={index}
            className="flex items-center justify-between py-2 px-3 bg-dark-darker rounded-lg"
          >
            <div className="flex items-center gap-3">
              <StatusIcon status={item.status} />
              <span className="font-medium text-text-light">{item.label}</span>
            </div>
            <span className="text-sm text-text-gray">{item.detail}</span>
          </div>
        ))}
      </div>
    </Panel>
  );
};
