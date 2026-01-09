import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig, i18n} = useDocusaurusContext();
  const isUrdu = i18n.currentLocale === 'ur';

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs">
            {isUrdu ? 'سیکھنا شروع کریں' : 'Start Learning'}
          </Link>
        </div>
      </div>
    </header>
  );
}

const features = {
  en: [
    {
      title: 'Physical AI & Robotics',
      icon: '🤖',
      description: 'Learn how AI systems interact with the physical world through sensors, actuators, and real-time decision making.',
    },
    {
      title: 'ROS 2 Framework',
      icon: '⚙️',
      description: 'Master the Robot Operating System 2 - the industry standard for building robotic applications.',
    },
    {
      title: 'Gazebo Simulation',
      icon: '🎮',
      description: 'Practice in realistic 3D simulations before deploying to real hardware.',
    },
    {
      title: 'AI-Powered Assistant',
      icon: '💬',
      description: 'Get instant answers from our RAG-powered chatbot trained on the entire textbook.',
    },
    {
      title: 'Personalized Learning',
      icon: '📊',
      description: 'Adaptive content that matches your skill level and learning preferences.',
    },
    {
      title: 'Bilingual Support',
      icon: '🌐',
      description: 'Full content available in English and Urdu with proper RTL support.',
    },
  ],
  ur: [
    {
      title: 'فزیکل AI اور روبوٹکس',
      icon: '🤖',
      description: 'سیکھیں کہ AI سسٹم سینسرز، ایکچویٹرز اور ریئل ٹائم فیصلہ سازی کے ذریعے فزیکل دنیا سے کیسے تعامل کرتے ہیں۔',
    },
    {
      title: 'ROS 2 فریم ورک',
      icon: '⚙️',
      description: 'روبوٹ آپریٹنگ سسٹم 2 میں مہارت حاصل کریں - روبوٹک ایپلیکیشنز بنانے کا صنعتی معیار۔',
    },
    {
      title: 'گزیبو سمولیشن',
      icon: '🎮',
      description: 'حقیقی ہارڈویئر پر تعیناتی سے پہلے 3D سمولیشنز میں مشق کریں۔',
    },
    {
      title: 'AI سے چلنے والا اسسٹنٹ',
      icon: '💬',
      description: 'پوری کتاب پر تربیت یافتہ چیٹ بوٹ سے فوری جوابات حاصل کریں۔',
    },
    {
      title: 'ذاتی نوعیت کا سیکھنا',
      icon: '📊',
      description: 'آپ کی مہارت کی سطح اور سیکھنے کی ترجیحات سے مطابقت رکھنے والا مواد۔',
    },
    {
      title: 'دو لسانی سپورٹ',
      icon: '🌐',
      description: 'مکمل مواد انگریزی اور اردو میں RTL سپورٹ کے ساتھ دستیاب۔',
    },
  ],
};

function Feature({icon, title, description}) {
  return (
    <div className={clsx('col col--4', styles.feature)}>
      <div className={styles.featureIcon}>{icon}</div>
      <h3>{title}</h3>
      <p>{description}</p>
    </div>
  );
}

function HomepageFeatures() {
  const {i18n} = useDocusaurusContext();
  const featureList = features[i18n.currentLocale] || features.en;

  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {featureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

function HomepageCTA() {
  const {i18n} = useDocusaurusContext();
  const isUrdu = i18n.currentLocale === 'ur';

  return (
    <section className={styles.cta}>
      <div className="container">
        <h2>{isUrdu ? 'مستقبل بنانے کے لیے تیار ہیں؟' : 'Ready to Build the Future?'}</h2>
        <p>{isUrdu ? 'آج ہی فزیکل AI اور روبوٹکس کا سفر شروع کریں۔' : 'Start your journey into Physical AI and robotics today.'}</p>
        <div className={styles.ctaButtons}>
          <Link
            className="button button--primary button--lg"
            to="/docs">
            {isUrdu ? 'شروع کریں' : 'Get Started'}
          </Link>
          <Link
            className="button button--outline button--primary button--lg"
            to="https://github.com/faiqahm/hackathon_I_book">
            {isUrdu ? 'GitHub پر دیکھیں' : 'View on GitHub'}
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Learn Physical AI, ROS 2, and Robotics with an AI-powered interactive textbook">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <HomepageCTA />
      </main>
    </Layout>
  );
}
