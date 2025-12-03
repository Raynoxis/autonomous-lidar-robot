/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        primary: {
          DEFAULT: '#2563eb',
          hover: '#1d4ed8',
        },
        success: {
          DEFAULT: '#10b981',
          hover: '#059669',
        },
        danger: {
          DEFAULT: '#ef4444',
          hover: '#dc2626',
        },
        warning: {
          DEFAULT: '#f59e0b',
        },
        dark: {
          bg: '#1f2937',
          darker: '#111827',
          card: '#374151',
          border: '#4b5563',
        },
        text: {
          light: '#f9fafb',
          gray: '#9ca3af',
        },
      },
    },
  },
  plugins: [],
}
