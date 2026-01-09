import React, { useState, useEffect } from 'react';
import { useAuth } from '@site/src/context/AuthContext';
import styles from './styles.module.css';

const API_BASE = '';

export default function OnboardingWizard({ isOpen, onClose, onComplete }) {
  const [questions, setQuestions] = useState([]);
  const [currentIndex, setCurrentIndex] = useState(0);
  const [answers, setAnswers] = useState({});
  const [loading, setLoading] = useState(true);
  const [submitting, setSubmitting] = useState(false);
  const { user } = useAuth();

  // Fetch questions on mount
  useEffect(() => {
    if (isOpen) {
      fetchQuestions();
    }
  }, [isOpen]);

  const fetchQuestions = async () => {
    try {
      const response = await fetch(`${API_BASE}/api/personalization/questions`);
      if (response.ok) {
        const data = await response.json();
        setQuestions(data);
      }
    } catch (error) {
      console.error('Failed to fetch questions:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleAnswer = (dimension, value) => {
    setAnswers(prev => ({ ...prev, [dimension]: value }));
  };

  const handleNext = () => {
    if (currentIndex < questions.length - 1) {
      setCurrentIndex(prev => prev + 1);
    }
  };

  const handlePrev = () => {
    if (currentIndex > 0) {
      setCurrentIndex(prev => prev - 1);
    }
  };

  const handleSubmit = async () => {
    setSubmitting(true);
    try {
      const response = await fetch(`${API_BASE}/api/personalization/profile`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          user_id: user?.user_id || `guest_${Date.now()}`,
          answers
        })
      });

      if (response.ok) {
        const profile = await response.json();
        // Store profile ID in localStorage
        localStorage.setItem('learner_profile_id', profile.user_id);
        localStorage.setItem('learner_profile', JSON.stringify(profile));
        onComplete?.(profile);
        onClose();
      }
    } catch (error) {
      console.error('Failed to save profile:', error);
    } finally {
      setSubmitting(false);
    }
  };

  if (!isOpen) return null;

  const currentQuestion = questions[currentIndex];
  const progress = ((currentIndex + 1) / questions.length) * 100;
  const canProceed = currentQuestion && answers[currentQuestion.dimension];
  const isLastQuestion = currentIndex === questions.length - 1;

  return (
    <div className={styles.overlay}>
      <div className={styles.wizard}>
        <div className={styles.header}>
          <h2>Personalize Your Learning</h2>
          <p>Answer {questions.length} quick questions to customize your experience</p>
        </div>

        {/* Progress Bar */}
        <div className={styles.progressContainer}>
          <div className={styles.progressBar} style={{ width: `${progress}%` }} />
          <span className={styles.progressText}>{currentIndex + 1} of {questions.length}</span>
        </div>

        {loading ? (
          <div className={styles.loading}>Loading questions...</div>
        ) : currentQuestion ? (
          <div className={styles.questionContainer}>
            <h3 className={styles.question}>{currentQuestion.question}</h3>

            <div className={styles.options}>
              {currentQuestion.options.map((option) => (
                <button
                  key={option.value}
                  className={`${styles.option} ${answers[currentQuestion.dimension] === option.value ? styles.selected : ''}`}
                  onClick={() => handleAnswer(currentQuestion.dimension, option.value)}
                >
                  <span className={styles.optionLabel}>{option.label}</span>
                  {answers[currentQuestion.dimension] === option.value && (
                    <span className={styles.checkmark}>✓</span>
                  )}
                </button>
              ))}
            </div>
          </div>
        ) : (
          <div className={styles.loading}>No questions available</div>
        )}

        {/* Navigation */}
        <div className={styles.navigation}>
          <button
            className={styles.navBtn}
            onClick={handlePrev}
            disabled={currentIndex === 0}
          >
            Previous
          </button>

          <button
            className={styles.skipBtn}
            onClick={onClose}
          >
            Skip for now
          </button>

          {isLastQuestion ? (
            <button
              className={`${styles.navBtn} ${styles.submitBtn}`}
              onClick={handleSubmit}
              disabled={!canProceed || submitting}
            >
              {submitting ? 'Saving...' : 'Complete Setup'}
            </button>
          ) : (
            <button
              className={`${styles.navBtn} ${styles.nextBtn}`}
              onClick={handleNext}
              disabled={!canProceed}
            >
              Next
            </button>
          )}
        </div>
      </div>
    </div>
  );
}
