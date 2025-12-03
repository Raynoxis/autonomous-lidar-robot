// Type definitions for roslib
declare module 'roslib' {
  export class Ros {
    constructor(options: { url: string });
    connect(url: string): void;
    close(): void;
    on(eventName: 'connection' | 'error' | 'close', callback: (event?: any) => void): void;
    getTopics(
      callback: (result: { topics: string[]; types: string[] }) => void,
      errorCallback?: (error: any) => void
    ): void;
    getNodes(
      callback: (nodes: string[]) => void,
      errorCallback?: (error: any) => void
    ): void;
    getTopicType(
      topic: string,
      callback: (type: string) => void,
      errorCallback?: (error: any) => void
    ): void;
    getServiceType(
      service: string,
      callback: (type: string) => void,
      errorCallback?: (error: any) => void
    ): void;
  }

  export class Topic<T = any> {
    constructor(options: {
      ros: Ros;
      name: string;
      messageType: string;
      throttle_rate?: number;
      queue_size?: number;
      latch?: boolean;
      queue_length?: number;
    });
    subscribe(callback: (message: T) => void): void;
    unsubscribe(): void;
    advertise(): void;
    unadvertise(): void;
    publish(message: T): void;
    _subscribeId?: number;
  }

  export class Service<TRequest = any, TResponse = any> {
    constructor(options: {
      ros: Ros;
      name: string;
      serviceType: string;
    });
    callService(
      request: TRequest,
      callback: (response: TResponse) => void,
      errorCallback?: (error: any) => void
    ): void;
  }

  export class ServiceRequest<T = any> {
    constructor(values?: Partial<T>);
  }

  export class ServiceResponse<T = any> {
    constructor(values?: Partial<T>);
  }

  export class Message<T = any> {
    constructor(values?: Partial<T>);
  }

  export class ActionClient {
    constructor(options: {
      ros: Ros;
      serverName: string;
      actionName: string;
      timeout?: number;
    });
    cancel(): void;
  }

  export class Goal<TGoal = any, TFeedback = any, TResult = any> {
    constructor(options: {
      actionClient: ActionClient;
      goalMessage: TGoal;
    });
    on(eventName: 'feedback', callback: (feedback: TFeedback) => void): void;
    on(eventName: 'result', callback: (result: TResult) => void): void;
    on(eventName: 'status', callback: (status: any) => void): void;
    on(eventName: 'error', callback: (error: any) => void): void;
    send(timeout?: number): void;
    cancel(): void;
  }

  export class Param {
    constructor(options: {
      ros: Ros;
      name: string;
    });
    get(callback: (value: any) => void): void;
    set(value: any, callback?: () => void): void;
    delete(callback?: () => void): void;
  }
}
