import './globals.css';
import type { Metadata } from 'next';

export const metadata: Metadata = {
  title: 'AERION MIND Console',
  description: 'Strategic planning console for AERION MIND'
};

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="ko">
      <body>{children}</body>
    </html>
  );
}
