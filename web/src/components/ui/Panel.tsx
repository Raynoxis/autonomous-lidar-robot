import React from 'react';

interface PanelProps {
  title?: string;
  children: React.ReactNode;
  className?: string;
  headerAction?: React.ReactNode;
}

export const Panel: React.FC<PanelProps> = ({ title, children, className = '', headerAction }) => {
  return (
    <div className={`bg-dark-bg rounded-xl border border-dark-border overflow-hidden flex flex-col ${className}`}>
      {title && (
        <div className="px-4 py-3 bg-dark-card border-b border-dark-border font-bold text-lg flex justify-between items-center">
          <span>{title}</span>
          {headerAction}
        </div>
      )}
      <div className="flex-1 overflow-auto p-4">
        {children}
      </div>
    </div>
  );
};
