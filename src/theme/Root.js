import React, { useState, useEffect } from 'react';
import { AuthProvider } from '@site/src/context/AuthContext';
import Chatbot from '@site/src/components/Chatbot';
import OnboardingWizard from '@site/src/components/OnboardingWizard';

// Root wrapper with auth, personalization, and chatbot
function RootContent({ children }) {
  const [showOnboarding, setShowOnboarding] = useState(false);

  useEffect(() => {
    // Check if user has completed onboarding
    const hasProfile = localStorage.getItem('learner_profile');
    const hasSeenOnboarding = localStorage.getItem('onboarding_seen');

    // Show onboarding after short delay for new visitors
    if (!hasProfile && !hasSeenOnboarding) {
      const timer = setTimeout(() => {
        setShowOnboarding(true);
      }, 3000); // Show after 3 seconds
      return () => clearTimeout(timer);
    }
  }, []);

  const handleOnboardingClose = () => {
    setShowOnboarding(false);
    localStorage.setItem('onboarding_seen', 'true');
  };

  const handleOnboardingComplete = (profile) => {
    setShowOnboarding(false);
    console.log('Profile created:', profile);
  };

  return (
    <>
      {children}
      <Chatbot />
      <OnboardingWizard
        isOpen={showOnboarding}
        onClose={handleOnboardingClose}
        onComplete={handleOnboardingComplete}
      />
    </>
  );
}

// Default implementation that wraps all pages with auth and chatbot
export default function Root({ children }) {
  return (
    <AuthProvider>
      <RootContent>{children}</RootContent>
    </AuthProvider>
  );
}
