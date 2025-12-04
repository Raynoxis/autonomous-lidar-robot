import React, { useState, useEffect, useRef } from 'react';
import { useRobotStore } from '../store';
import { Panel } from './ui';

interface StatusItemProps {
  label: string;
  active: boolean;
  highlight?: boolean;
}

const StatusItem: React.FC<StatusItemProps> = ({ label, active, highlight }) => {
  return (
    <div
      className={`flex items-center justify-between py-1.5 px-2 rounded transition-all duration-300 ${
        highlight ? 'bg-warning/20 animate-pulse' : ''
      }`}
    >
      <span className="text-sm text-text-light">{label}</span>
      <div
        className={`w-2 h-2 rounded-full transition-colors ${
          active ? 'bg-success' : 'bg-gray-600'
        }`}
      />
    </div>
  );
};

interface CollapsibleSectionProps {
  title: string;
  count: { active: number; total: number };
  children: React.ReactNode;
}

const CollapsibleSection: React.FC<CollapsibleSectionProps> = ({ title, count, children }) => {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <div className="border border-dark-border rounded-lg overflow-hidden">
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="w-full px-3 py-2 bg-dark-card hover:bg-dark-border flex items-center justify-between transition-colors"
      >
        <span className="font-medium text-text-light">{title}</span>
        <div className="flex items-center gap-2">
          <span className="text-xs text-text-gray">
            {count.active}/{count.total}
          </span>
          <span className={`transform transition-transform ${isOpen ? 'rotate-180' : ''}`}>
            ▼
          </span>
        </div>
      </button>
      {isOpen && (
        <div className="px-2 py-2 bg-dark-darker max-h-60 overflow-y-auto space-y-0.5">
          {children}
        </div>
      )}
    </div>
  );
};

export const SystemStatus: React.FC = () => {
  const { systemState, nodes, topics, scanDataReceived, connected } = useRobotStore();

  // Track changes for highlighting
  const [highlightedNodes, setHighlightedNodes] = useState<Set<string>>(new Set());
  const [highlightedTopics, setHighlightedTopics] = useState<Set<string>>(new Set());
  const prevNodesRef = useRef<Record<string, boolean>>(nodes);
  const prevTopicsRef = useRef<Record<string, boolean>>(topics);

  // Detect changes and highlight
  useEffect(() => {
    const changedNodes = new Set<string>();
    const changedTopics = new Set<string>();

    // Check nodes changes
    Object.keys(nodes).forEach((node) => {
      if (nodes[node] !== prevNodesRef.current[node]) {
        changedNodes.add(node);
      }
    });

    // Check topics changes
    Object.keys(topics).forEach((topic) => {
      if (topics[topic] !== prevTopicsRef.current[topic]) {
        changedTopics.add(topic);
      }
    });

    if (changedNodes.size > 0) {
      setHighlightedNodes(changedNodes);
      setTimeout(() => setHighlightedNodes(new Set()), 2000);
    }

    if (changedTopics.size > 0) {
      setHighlightedTopics(changedTopics);
      setTimeout(() => setHighlightedTopics(new Set()), 2000);
    }

    prevNodesRef.current = nodes;
    prevTopicsRef.current = topics;
  }, [nodes, topics]);

  // Core system components
  const coreComponents = [
    {
      label: 'WebSocket',
      active: connected,
    },
    {
      label: 'ROS Core',
      active: connected && Object.values(nodes).some((n) => n),
    },
    {
      label: 'Nav2 Stack',
      active:
        nodes['/bt_navigator'] &&
        nodes['/controller_server'] &&
        nodes['/planner_server'],
    },
    {
      label: 'SLAM',
      active: nodes['/slam_toolbox'],
    },
    {
      label: 'Robot Hardware',
      active: scanDataReceived && nodes['/kaiaai_telemetry_node'],
    },
  ];

  // Sort nodes/topics: active first, then highlighted, then alphabetically
  const sortItems = <T extends string>(
    items: Record<T, boolean>,
    highlighted: Set<string>
  ): T[] => {
    return Object.keys(items)
      .sort((a, b) => {
        // Active items first
        if (items[a as T] !== items[b as T]) {
          return items[a as T] ? -1 : 1;
        }
        // Then highlighted
        if (highlighted.has(a) !== highlighted.has(b)) {
          return highlighted.has(a) ? -1 : 1;
        }
        // Then alphabetical
        return a.localeCompare(b);
      }) as T[];
  };

  const sortedNodes = sortItems(nodes, highlightedNodes);
  const sortedTopics = sortItems(topics, highlightedTopics);

  const nodesCount = {
    active: Object.values(nodes).filter(Boolean).length,
    total: Object.keys(nodes).length,
  };

  const topicsCount = {
    active: Object.values(topics).filter(Boolean).length,
    total: Object.keys(topics).length,
  };

  return (
    <Panel title="État Système" className="overflow-hidden">
      <div className="space-y-3">
        {/* Core System Components */}
        <div className="space-y-1">
          <h3 className="text-xs font-bold text-text-gray uppercase mb-2">
            Composants Centraux
          </h3>
          {coreComponents.map((component, index) => (
            <div
              key={index}
              className="flex items-center justify-between py-2 px-3 bg-dark-darker rounded-lg"
            >
              <span className="text-sm font-medium text-text-light">{component.label}</span>
              <div
                className={`w-3 h-3 rounded-full transition-colors ${
                  component.active ? 'bg-success' : 'bg-gray-600'
                }`}
              />
            </div>
          ))}
        </div>

        {/* ROS Nodes */}
        <CollapsibleSection title="ROS Nodes" count={nodesCount}>
          {sortedNodes.map((node) => (
            <StatusItem
              key={node}
              label={node}
              active={nodes[node]}
              highlight={highlightedNodes.has(node)}
            />
          ))}
        </CollapsibleSection>

        {/* ROS Topics */}
        <CollapsibleSection title="ROS Topics" count={topicsCount}>
          {sortedTopics.map((topic) => (
            <StatusItem
              key={topic}
              label={topic}
              active={topics[topic]}
              highlight={highlightedTopics.has(topic)}
            />
          ))}
        </CollapsibleSection>

        {/* System State Indicator */}
        <div className="pt-2 border-t border-dark-border">
          <div className="text-xs text-text-gray text-center">
            État: <span className="font-mono text-text-light">{systemState}</span>
          </div>
        </div>
      </div>
    </Panel>
  );
};
