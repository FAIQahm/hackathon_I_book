import React from 'react';
import { AuthProvider } from '@site/src/context/AuthContext';
import Chatbot from '@site/src/components/Chatbot';

// Default implementation that wraps all pages with auth and chatbot
export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
      <Chatbot />
    </AuthProvider>
  );
}
