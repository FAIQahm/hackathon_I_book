import React from 'react';
import Chatbot from '@site/src/components/Chatbot';

// Default implementation that wraps all pages with the chatbot
export default function Root({ children }) {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}
