export function YoutubeFooter() {
  return (
    <footer className="mt-4 bg-dark-card border-t border-dark-border p-3 flex items-center justify-center gap-3 text-text-gray">
      <a
        href="https://www.youtube.com/@Raynoxis"
        target="_blank"
        rel="noreferrer"
        className="flex items-center gap-2 hover:text-text-light transition-colors"
      >
        <img
          src="https://yt3.googleusercontent.com/QWhV5h8aevOT-vQkxpBIlLbvFIg19hS3HCBKQtKsRMHbp2opxUra0xMPmB1pkMBM2i2Y6NFNN9M=s160-c-k-c0x00ffffff-no-rj"
          alt="Raynoxis YouTube avatar"
          className="w-8 h-8 rounded-full border border-dark-border"
          loading="lazy"
        />
        <span className="text-sm font-medium">Raynoxis · YouTube</span>
      </a>
    </footer>
  );
}
